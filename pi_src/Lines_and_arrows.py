import cv2
import numpy as np
import serial
import time
from collections import deque
from picamera2 import Picamera2

# ------------------------------
# Serial settings (to Pico)
# ------------------------------
PORT = "/dev/ttyACM0"
BAUD = 115200
ser = serial.Serial(PORT, BAUD, timeout=1)

# -----------------------------
# Camera settings
# -----------------------------
FRAME_W = 960
FRAME_H = 540
FPS_TARGET = 40

picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={"format": "RGB888", "size": (FRAME_W, FRAME_H)},
    controls={"FrameRate": FPS_TARGET},
    raw={"size": (2304, 1296)}
)

picam2.configure(config)

picam2.set_controls({
    "ExposureTime": 13000,
    "AnalogueGain": 2.8,
    "ColourGains": (2.1, 1.6)
})

picam2.start()

# -----------------------------
# HSV thresholds
# -----------------------------

# Yellow line, left side
YELLOW_LOWER = np.array([19, 176, 108])
YELLOW_UPPER = np.array([29, 255, 182])

# Blue line, right side
BLUE_LOWER = np.array([97, 23, 11])
BLUE_UPPER = np.array([179, 177, 115])

# Arrow thresholds from your arrow detection code
ARROW_LOWER = np.array([10, 97, 15])
ARROW_UPPER = np.array([31, 172, 120])

# -----------------------------
# Line following settings
# -----------------------------
LOOKAHEAD_Y = int(FRAME_H * 0.55)
BAND_HEIGHT = 80
MIN_PIXELS_LINE = 50

last_error = 0.0
last_lane_width = None

base_speed = 40

# -----------------------------
# Arrow detection settings
# -----------------------------

# Minimum arrow blob size
MIN_AREA_FRAC = 0.003
MIN_AREA = int(FRAME_W * FRAME_H * MIN_AREA_FRAC)

# Mask cleaning
ARROW_KERNEL = np.ones((3, 3), np.uint8)

# How far the arrow tip needs to be from the centre
MIN_TIP_OFFSET_FRAC = 0.12

# Only search for arrows in this region of the image
# This helps stop random background objects being detected
ARROW_ROI_X1 = int(FRAME_W * 0.10)
ARROW_ROI_X2 = int(FRAME_W * 0.90)
ARROW_ROI_Y1 = int(FRAME_H * 0.20)
ARROW_ROI_Y2 = int(FRAME_H * 0.90)

# Basic shape filtering
MIN_ARROW_W = 35
MIN_ARROW_H = 25
MIN_FILL_RATIO = 0.12
MAX_FILL_RATIO = 0.85
MIN_ASPECT_RATIO = 0.5
MAX_ASPECT_RATIO = 4.0

# Arrow bias timing
ARROW_STRONG_TIME = 0.5
ARROW_MODERATE_TIME = 0.5
ARROW_WEAK_TIME = 0.5

# Arrow bias strength
# These values are normalised error values.
# 0.95 becomes about 95 after multiplying by 100.
ARROW_STRONG_BIAS = 0.95
ARROW_MODERATE_BIAS = 0.60
ARROW_WEAK_BIAS = 0.30

# Positive error should turn right.
# If the robot turns the wrong way, change this to -1.
ARROW_RIGHT_SIGN = 1

# Require a direction to appear a couple of times recently before triggering
recent_arrow_directions = deque(maxlen=6)
ARROW_HITS_TO_TRIGGER = 2

# Prevent the same arrow from instantly retriggering
ARROW_COOLDOWN_TIME = 1.0

arrow_bias_start_time = None
arrow_bias_direction = None
last_arrow_trigger_time = -999.0

# -----------------------------
# FPS settings
# -----------------------------
prev_time = time.time()
fps = 0.0


# ============================================================
# Helper functions
# ============================================================

def mask_and_clean(hsv, lower, upper, kernel_size=5):
    """
    Creates a colour mask and cleans small noise.
    Used for the yellow and blue line masks.
    """

    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    return mask


def get_line_x(mask, y, band_height):
    """
    Finds the median x position of a line inside a horizontal band.
    Returns None if not enough pixels are found.
    """

    y1 = max(0, y - band_height // 2)
    y2 = min(mask.shape[0], y + band_height // 2)

    band = mask[y1:y2, :]
    ys, xs = np.where(band > 0)

    if len(xs) < MIN_PIXELS_LINE:
        return None

    return int(np.median(xs))


def get_arrow_mask(hsv):
    """
    Creates the arrow mask.
    The ROI prevents the detector from looking at the whole image.
    """

    mask = cv2.inRange(hsv, ARROW_LOWER, ARROW_UPPER)

    # Keep only the arrow search region
    roi_mask = np.zeros_like(mask)
    roi_mask[ARROW_ROI_Y1:ARROW_ROI_Y2, ARROW_ROI_X1:ARROW_ROI_X2] = 255
    mask = cv2.bitwise_and(mask, roi_mask)

    # Clean noise and fill small holes
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, ARROW_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, ARROW_KERNEL)

    return mask


def find_largest_arrow_contour(mask):
    """
    Finds the largest contour that looks roughly arrow-sized.
    This filters out tiny blobs and weird-shaped detections.
    """

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None

    valid_contours = []

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < MIN_AREA:
            continue

        x, y, w, h = cv2.boundingRect(contour)

        if w < MIN_ARROW_W or h < MIN_ARROW_H:
            continue

        aspect_ratio = w / h
        if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
            continue

        fill_ratio = area / (w * h)
        if fill_ratio < MIN_FILL_RATIO or fill_ratio > MAX_FILL_RATIO:
            continue

        valid_contours.append(contour)

    if not valid_contours:
        return None

    return max(valid_contours, key=cv2.contourArea)


def detect_arrow_direction(contour):
    """
    Detects whether the arrow points LEFT or RIGHT.
    Returns LEFT, RIGHT, or NOT_ARROW.
    """

    if contour is None or len(contour) < 5:
        return "NOT_ARROW"

    x, y, w, h = cv2.boundingRect(contour)

    if w <= 0 or h <= 0:
        return "NOT_ARROW"

    M = cv2.moments(contour)

    if M["m00"] == 0:
        return "NOT_ARROW"

    # Centre of the detected shape
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    points = contour[:, 0, :]

    # Look for the arrow tip in the upper part of the contour
    upper_limit = y + int(0.65 * h)
    upper_points = points[points[:, 1] < upper_limit]

    if len(upper_points) == 0:
        return "NOT_ARROW"

    # The tip should be the upper point farthest from the centre
    distances = (upper_points[:, 0] - cx) ** 2 + (upper_points[:, 1] - cy) ** 2
    tip = upper_points[np.argmax(distances)]

    tip_x = int(tip[0])
    dx = tip_x - cx

    # If the tip is not far enough left/right, do not trust it
    if abs(dx) < MIN_TIP_OFFSET_FRAC * w:
        return "NOT_ARROW"

    elif dx > 0:
        return "RIGHT"

    else:
        return "LEFT"


def get_stable_arrow_direction(raw_direction):
    """
    Uses a small history so the arrow does not need to be detected
    perfectly every frame while the robot is moving.
    """

    if raw_direction == "LEFT" or raw_direction == "RIGHT":
        recent_arrow_directions.append(raw_direction)
    else:
        recent_arrow_directions.append("NONE")

    left_hits = recent_arrow_directions.count("LEFT")
    right_hits = recent_arrow_directions.count("RIGHT")

    if left_hits >= ARROW_HITS_TO_TRIGGER and left_hits > right_hits:
        return "LEFT"

    if right_hits >= ARROW_HITS_TO_TRIGGER and right_hits > left_hits:
        return "RIGHT"

    return "NOT_ARROW"


def get_arrow_bias(current_time):
    """
    Returns the current arrow bias based on how long ago the arrow was triggered.
    First 0.5 s: strong
    Next 0.5 s: moderate
    Next 0.5 s: weak
    """

    global arrow_bias_start_time
    global arrow_bias_direction

    if arrow_bias_start_time is None:
        return 0.0

    elapsed = current_time - arrow_bias_start_time

    if elapsed < ARROW_STRONG_TIME:
        bias_strength = ARROW_STRONG_BIAS

    elif elapsed < ARROW_STRONG_TIME + ARROW_MODERATE_TIME:
        bias_strength = ARROW_MODERATE_BIAS

    elif elapsed < ARROW_STRONG_TIME + ARROW_MODERATE_TIME + ARROW_WEAK_TIME:
        bias_strength = ARROW_WEAK_BIAS

    else:
        arrow_bias_start_time = None
        arrow_bias_direction = None
        return 0.0

    if arrow_bias_direction == "RIGHT":
        return ARROW_RIGHT_SIGN * bias_strength

    if arrow_bias_direction == "LEFT":
        return -ARROW_RIGHT_SIGN * bias_strength

    return 0.0


# ============================================================
# Main loop
# ============================================================

try:
    while True:
        frame = picam2.capture_array()

        # Calculate FPS
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        if dt > 0:
            fps = 1.0 / dt

        # Keeping this the same as your working code.
        # Even though the camera format says RGB888, your thresholds were tuned with this conversion.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # -----------------------------
        # Line following
        # -----------------------------

        yellow_mask = mask_and_clean(hsv, YELLOW_LOWER, YELLOW_UPPER)
        blue_mask = mask_and_clean(hsv, BLUE_LOWER, BLUE_UPPER)

        yellow_x = get_line_x(yellow_mask, LOOKAHEAD_Y, BAND_HEIGHT)
        blue_x = get_line_x(blue_mask, LOOKAHEAD_Y, BAND_HEIGHT)

        lane_centre_x = None

        if yellow_x is not None and blue_x is not None:
            lane_centre_x = (yellow_x + blue_x) / 2

            measured_width = blue_x - yellow_x
            if measured_width > 0:
                last_lane_width = measured_width

        elif yellow_x is not None and last_lane_width is not None:
            lane_centre_x = yellow_x + last_lane_width / 2

        elif blue_x is not None and last_lane_width is not None:
            lane_centre_x = blue_x - last_lane_width / 2

        if lane_centre_x is not None:
            image_centre_x = FRAME_W / 2

            error_px = lane_centre_x - image_centre_x
            error = error_px / (FRAME_W / 2)

            # Smooth the line error
            error = 0.7 * last_error + 0.3 * error
            last_error = error

        else:
            # If no line is found, keep the previous error
            error = last_error

        # -----------------------------
        # Arrow detection
        # -----------------------------

        arrow_mask = get_arrow_mask(hsv)
        arrow_contour = find_largest_arrow_contour(arrow_mask)
        raw_arrow_direction = detect_arrow_direction(arrow_contour)
        stable_arrow_direction = get_stable_arrow_direction(raw_arrow_direction)

        # Trigger arrow bias only if:
        # - a stable arrow was seen
        # - no arrow bias is currently active
        # - cooldown has passed
        if stable_arrow_direction != "NOT_ARROW":
            cooldown_done = current_time - last_arrow_trigger_time > ARROW_COOLDOWN_TIME

            if arrow_bias_start_time is None and cooldown_done:
                arrow_bias_start_time = current_time
                arrow_bias_direction = stable_arrow_direction
                last_arrow_trigger_time = current_time
                recent_arrow_directions.clear()

                print("Arrow triggered:", stable_arrow_direction)

        arrow_bias = get_arrow_bias(current_time)

        # Add arrow correction to normal line following
        final_error = error + arrow_bias

        # Convert to integer for Pico
        error_int = int(final_error * 100)
        error_int = max(-100, min(100, error_int))

        # Send command to Pico
        msg = f"D,{error_int},{base_speed}\n"
        ser.write(msg.encode("utf-8"))

        print(error_int)

        # -----------------------------
        # Debug display
        # -----------------------------

        display = frame.copy()
        mask_display = np.zeros_like(display)

        # Show masks in colour
        mask_display[yellow_mask > 0] = (0, 255, 255)
        mask_display[blue_mask > 0] = (255, 0, 0)
        mask_display[arrow_mask > 0] = (0, 255, 0)

        # Draw arrow contour only
        if arrow_contour is not None:
            cv2.drawContours(display, [arrow_contour], -1, (0, 255, 0), 2)

        # Lookahead line
        cv2.line(display, (0, LOOKAHEAD_Y), (FRAME_W, LOOKAHEAD_Y), (255, 255, 255), 2)
        cv2.line(mask_display, (0, LOOKAHEAD_Y), (FRAME_W, LOOKAHEAD_Y), (255, 255, 255), 2)

        # Image centre line
        cv2.line(display, (FRAME_W // 2, 0), (FRAME_W // 2, FRAME_H), (255, 255, 255), 2)

        # Draw detected line positions
        if yellow_x is not None:
            cv2.circle(display, (yellow_x, LOOKAHEAD_Y), 8, (0, 255, 255), -1)

        if blue_x is not None:
            cv2.circle(display, (blue_x, LOOKAHEAD_Y), 8, (255, 0, 0), -1)

        if lane_centre_x is not None:
            cv2.circle(display, (int(lane_centre_x), LOOKAHEAD_Y), 8, (0, 255, 0), -1)

        # Text display
        cv2.putText(display, f"Final error: {error_int}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.putText(display, f"Line error: {int(error * 100)}", (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.putText(display, f"Raw arrow: {raw_arrow_direction}", (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0) if raw_arrow_direction != "NOT_ARROW" else (0, 0, 255), 2)

        cv2.putText(display, f"Bias: {int(arrow_bias * 100)}", (20, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.putText(display, f"FPS: {fps:.1f}", (20, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        combined = np.hstack((display, mask_display))
        cv2.imshow("Lane + Arrow Detection | Masks", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break

finally:
    # Stop robot safely
    msg = f"D,{0},{0}\n"
    ser.write(msg.encode("utf-8"))

    ser.close()
    cv2.destroyAllWindows()
    picam2.stop()