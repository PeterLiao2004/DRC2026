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

# picam2.set_controls({
#     "ExposureTime": 13000,
#     "AnalogueGain": 2.8,
#     "ColourGains": (2.1, 1.6)
# })
picam2.set_controls({"ExposureTime": 9000, "AnalogueGain": 2.0, "ColourGains": (2.1, 1.9)})

picam2.start()

# -----------------------------
# HSV thresholds
# -----------------------------

# SOFIA HOUSE:
# # Yellow line, left side
# YELLOW_LOWER = np.array([19, 176, 108])
# YELLOW_UPPER = np.array([29, 255, 182])

# BLUE_LOWER = np.array([0, 0, 0])
# BLUE_UPPER = np.array([171, 206, 70])

# # Arrow thresholds from your arrow detection code
# ARROW_LOWER = np.array([10, 97, 15])
# ARROW_UPPER = np.array([31, 172, 120])

# # Green Line, start/stop
# GREEN_LOWER = np.array([19, 134, 19])
# GREEN_UPPER = np.array([59, 209, 104])

# UNI TRACK:
YELLOW_LOWER = np.array([24, 41, 179])
YELLOW_UPPER = np.array([32, 167, 255])

BLUE_LOWER = np.array([59, 0, 143])
BLUE_UPPER = np.array([120, 236, 255])

GREEN_LOWER = np.array([41, 28, 183])
GREEN_UPPER = np.array([84, 129, 255])

PURPLE_LOWER = np.array([155, 73, 70])
PURPLE_UPPER = np.array([179, 173, 166])

ARROW_LOWER = np.array([27, 0, 42])
ARROW_UPPER = np.array([107, 48, 162])

# -----------------------------
# Line following settings
# -----------------------------
LOOKAHEAD_Y = int(FRAME_H * 0.50)
BAND_HEIGHT = 100
MIN_PIXELS_LINE = 50

last_error = 0.0
last_lane_width = None

current_speed = 35
base_speed = 40
arrow_confirming_speed = 10
arrow_speed = 25

Kp = 0.28
Ki = 0
Kd = 0.1

green_seen_start_time = None
GREEN_STOP_DELAY = 2.5
robot_stopped_by_green = False

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
ARROW_ROI_Y1 = int(FRAME_H * 0.45)
ARROW_ROI_Y2 = int(FRAME_H * 1.00)

# Basic shape filtering
MIN_ARROW_W = 35
MIN_ARROW_H = 25
MIN_FILL_RATIO = 0.12
MAX_FILL_RATIO = 0.85
MIN_ASPECT_RATIO = 0.5
MAX_ASPECT_RATIO = 4.0

# Arrow bias timing
ARROW_WAIT_TIME = 2.2
ARROW_STRONG_TIME = 1.3
ARROW_MODERATE_TIME = 1.1
ARROW_WEAK_TIME = 1.5

# Arrow bias strength
# These values are normalised error values.
# 0.95 becomes about 95 after multiplying by 100.
ARROW_WAIT_BIAS = 0.02
ARROW_STRONG_BIAS = 0.8
ARROW_MODERATE_BIAS = 0.50
ARROW_WEAK_BIAS = 0.25

# Positive error should turn right.
# If the robot turns the wrong way, change this to -1.
ARROW_RIGHT_SIGN = 1

# Arrow confirmation system
ARROW_CONFIRM_FRAMES = 5          # same direction must be seen this many frames
ARROW_CONFIRM_TIMEOUT = 1.0       # cancel confirmation if it takes too long
ARROW_LOST_TIMEOUT = 0.25         # allow brief missed frames while confirming

# Prevent the same arrow from instantly retriggering
ARROW_COOLDOWN_TIME = 1.0

# Pending arrow confirmation state
pending_arrow_direction = None
pending_arrow_count = 0
pending_arrow_start_time = None
last_pending_arrow_seen_time = None

# Confirmed arrow bias state
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


def get_arrow_mask(hsv, blue_mask):
    """
    Creates the arrow mask, but removes any areas that overlap with the blue line.
    This stops the blue line being detected as part of the arrow.
    """

    # Normal arrow colour threshold
    mask = cv2.inRange(hsv, ARROW_LOWER, ARROW_UPPER)

    # Keep only the arrow search region
    roi_mask = np.zeros_like(mask)
    roi_mask[ARROW_ROI_Y1:ARROW_ROI_Y2, ARROW_ROI_X1:ARROW_ROI_X2] = 255
    mask = cv2.bitwise_and(mask, roi_mask)

    # Dilate the blue mask slightly so we remove the blue line edges too
    blue_ignore_kernel = np.ones((7, 7), np.uint8)
    blue_ignore_mask = cv2.dilate(blue_mask, blue_ignore_kernel, iterations=1)

    # Remove anything from the arrow mask that overlaps with the blue line
    mask = cv2.bitwise_and(mask, cv2.bitwise_not(blue_ignore_mask))

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


def get_arrow_bias(current_time):
    """
    Returns the current arrow bias after an arrow has been confirmed.
    """

    global arrow_bias_start_time
    global arrow_bias_direction

    if arrow_bias_start_time is None:
        return 0.0

    elapsed = current_time - arrow_bias_start_time

    if elapsed < ARROW_WAIT_TIME:
        bias_strength = ARROW_WAIT_BIAS

    elif elapsed < ARROW_WAIT_TIME + ARROW_STRONG_TIME:
        bias_strength = ARROW_STRONG_BIAS

    elif elapsed < ARROW_WAIT_TIME + ARROW_STRONG_TIME + ARROW_MODERATE_TIME:
        bias_strength = ARROW_MODERATE_BIAS

    elif elapsed < ARROW_WAIT_TIME + ARROW_STRONG_TIME + ARROW_MODERATE_TIME + ARROW_WEAK_TIME:
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

def detect_green_finish_line(green_mask):
    """
    Detects a wide green line across the track.
    This avoids stopping for small green progress markers.
    """

    # Look near the lower/middle part of the image
    y1 = int(FRAME_H * 0.45)
    y2 = int(FRAME_H * 0.85)

    roi = green_mask[y1:y2, :]

    contours, _ = cv2.findContours(
        roi,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return False, None

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    x, y, w, h = cv2.boundingRect(largest)

    # Shift y back to full frame coordinates
    y = y + y1

    # Tune these
    MIN_GREEN_AREA = 800
    MIN_GREEN_WIDTH = int(FRAME_W * 0.25)

    # A finish/start line should be reasonably wide
    if area > MIN_GREEN_AREA and w > MIN_GREEN_WIDTH:
        return True, (x, y, w, h)

    return False, (x, y, w, h)

def update_arrow_confirmation(raw_direction, current_time):
    """
    First slows down when an arrow candidate is seen.
    Only confirms the arrow if the same direction is detected for several frames.
    """

    global pending_arrow_direction
    global pending_arrow_count
    global pending_arrow_start_time
    global last_pending_arrow_seen_time

    global arrow_bias_start_time
    global arrow_bias_direction
    global last_arrow_trigger_time

    arrow_confirmed_now = False

    # If arrow bias is already active, do not start confirming another arrow
    if arrow_bias_start_time is not None:
        return False, False

    # Cooldown after a confirmed arrow
    if current_time - last_arrow_trigger_time < ARROW_COOLDOWN_TIME:
        return False, False

    valid_direction = raw_direction == "LEFT" or raw_direction == "RIGHT"

    # No arrow seen this frame
    if not valid_direction:
        if pending_arrow_start_time is not None:
            lost_time = current_time - last_pending_arrow_seen_time

            # Cancel if it has been missing for too long
            if lost_time > ARROW_LOST_TIMEOUT:
                pending_arrow_direction = None
                pending_arrow_count = 0
                pending_arrow_start_time = None
                last_pending_arrow_seen_time = None

        return False, pending_arrow_start_time is not None

    # First arrow candidate seen
    if pending_arrow_direction is None:
        pending_arrow_direction = raw_direction
        pending_arrow_count = 1
        pending_arrow_start_time = current_time
        last_pending_arrow_seen_time = current_time

        print("Arrow candidate:", raw_direction)

        # True means slow down while confirming
        return False, True

    # Same direction seen again
    if raw_direction == pending_arrow_direction:
        pending_arrow_count += 1
        last_pending_arrow_seen_time = current_time

        print("Arrow confirming:", pending_arrow_direction, pending_arrow_count)

    # Different direction seen, restart confirmation
    else:
        pending_arrow_direction = raw_direction
        pending_arrow_count = 1
        pending_arrow_start_time = current_time
        last_pending_arrow_seen_time = current_time

        print("Arrow direction changed, restarting:", raw_direction)

        return False, True

    # Cancel if confirmation takes too long
    if current_time - pending_arrow_start_time > ARROW_CONFIRM_TIMEOUT:
        print("Arrow confirmation timed out")

        pending_arrow_direction = None
        pending_arrow_count = 0
        pending_arrow_start_time = None
        last_pending_arrow_seen_time = None

        return False, False

    # Confirm arrow after enough same-direction frames
    if pending_arrow_count >= ARROW_CONFIRM_FRAMES:
        arrow_bias_start_time = current_time
        arrow_bias_direction = pending_arrow_direction
        last_arrow_trigger_time = current_time

        print("Arrow confirmed:", arrow_bias_direction)

        pending_arrow_direction = None
        pending_arrow_count = 0
        pending_arrow_start_time = None
        last_pending_arrow_seen_time = None

        arrow_confirmed_now = True

    return arrow_confirmed_now, pending_arrow_start_time is not None

# ============================================================
# Main loop
# ============================================================

try:
    # Send PID settings to Pico
    msg = f"PID,{Kp},{Ki},{Kd}\n"
    ser.write(msg.encode("utf-8"))

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
        green_mask = mask_and_clean(hsv, GREEN_LOWER, GREEN_UPPER)

        finish_line_seen, green_box = detect_green_finish_line(green_mask)

        # -----------------------------
        # Green finish line stop delay
        # -----------------------------
        if finish_line_seen and green_seen_start_time is None:
            green_seen_start_time = current_time
            print("Green line first seen - still following line")

        # Keep following yellow/blue while the green timer counts down
        if green_seen_start_time is not None:
            green_elapsed = current_time - green_seen_start_time
            green_remaining = GREEN_STOP_DELAY - green_elapsed

            if green_elapsed >= GREEN_STOP_DELAY:
                print("Green delay finished - stopping")
                ser.write(b"D,0,0\n")
                robot_stopped_by_green = True
                break

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

        arrow_mask = get_arrow_mask(hsv, blue_mask)
        arrow_contour = find_largest_arrow_contour(arrow_mask)
        raw_arrow_direction = detect_arrow_direction(arrow_contour)

        arrow_confirmed_now, arrow_confirming = update_arrow_confirmation(
            raw_arrow_direction,
            current_time
        )

        arrow_bias = get_arrow_bias(current_time)

        # Add arrow correction to normal line following
        final_error = error + arrow_bias

        # Convert to integer for Pico
        error_int = int(final_error * 100)
        error_int = max(-100, min(100, error_int))

        # Slow down while checking or executing an arrow
        if arrow_confirming:
            current_speed = arrow_confirming_speed
        elif arrow_bias != 0:
            current_speed = arrow_speed
        else:
            current_speed = base_speed

        # Send command to Pico
        msg = f"D,{error_int},{current_speed}\n"
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

        cv2.putText(display, f"Confirming: {arrow_confirming}", (20, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0) if arrow_confirming else (0, 0, 255), 2)

        cv2.putText(display, f"Confirm count: {pending_arrow_count}", (20, 280),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2)

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