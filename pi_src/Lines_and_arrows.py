import cv2
import numpy as np
import serial
import time
from picamera2 import Picamera2

# ------------------------------
# Serial settings (to pico)
# ------------------------------
PORT = "/dev/ttyACM0"
BAUD = 115200
ser = serial.Serial(PORT, BAUD, timeout=1)

# -----------------------------
# Camera settings
# -----------------------------

FRAME_W = 960
FRAME_H = 540

picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={"format": "RGB888", "size": (FRAME_W, FRAME_H)},
    controls={"FrameRate": 40},
    raw={"size": (2304, 1296)}
)

picam2.configure(config)

# Put your tuned settings here
picam2.set_controls({
    "ExposureTime": 13000,
    "AnalogueGain": 2.8,
    "ColourGains": (2.1, 1.6)
})

picam2.start()

# -----------------------------
# HSV thresholds - tune these values
# -----------------------------

# Yellow line, left side
YELLOW_LOWER = np.array([19, 176, 108])
YELLOW_UPPER = np.array([29, 255, 182])

# Blue line, right side
BLUE_LOWER = np.array([97, 23, 11])
BLUE_UPPER = np.array([179, 177, 115])

# Green line, start/stop
GREEN_LOWER = np.array([40, 50, 50])
GREEN_UPPER = np.array([80, 255, 255])

# Black arrow, for turning
# Using thresholds from the separate arrow detection code
ARROW_LOWER = np.array([10, 97, 15])
ARROW_UPPER = np.array([31, 172, 120])

# -----------------------------
# Line detection settings
# -----------------------------

LOOKAHEAD_Y = int(FRAME_H * 0.55)   # where to look for the lines
BAND_HEIGHT = 80                    # thickness of horizontal band
MIN_PIXELS_LINE = 50                # minimum pixels needed to trust a line

last_error = 0.0
last_lane_width = None

base_speed = 30  # Base speed to send to microcontroller (0-100)

# -----------------------------
# Arrow detection settings
# -----------------------------

MIN_AREA_FRAC = 0.005
MIN_AREA = int(FRAME_W * FRAME_H * MIN_AREA_FRAC)

ARROW_KERNEL_SIZE = 3
ARROW_KERNEL = np.ones((ARROW_KERNEL_SIZE, ARROW_KERNEL_SIZE), np.uint8)

MIN_TIP_OFFSET_FRAC = 0.10

# Arrow error bias timing
ARROW_STRONG_TIME = 0.5
ARROW_MODERATE_TIME = 0.5
ARROW_WEAK_TIME = 0.5

# Bias strengths are in normalised error units
# These become about +65, +35, +15 once multiplied by 100
ARROW_STRONG_BIAS = 0.65
ARROW_MODERATE_BIAS = 0.35
ARROW_WEAK_BIAS = 0.15

# Positive error is assumed to bias right
# If arrow turning is backwards, change this to -1
ARROW_RIGHT_SIGN = 1

arrow_bias_start_time = None
arrow_bias_direction = None
arrow_visible_last = False
last_arrow_print_direction = None

# -----------------------------
# FPS settings
# -----------------------------
prev_time = time.time()
fps = 0.0


def mask_and_clean(hsv, lower, upper, kernel_size=5):
    """
    Creates a binary mask for the given HSV range and applies morphological opening to clean it.
    """

    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    cleaned_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    return cleaned_mask


def get_line_x(mask, y, band_height):
    """
    Finds the average x position of a line inside a horizontal band.
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
    Creates and cleans the arrow mask.
    """

    mask = cv2.inRange(hsv, ARROW_LOWER, ARROW_UPPER)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, ARROW_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, ARROW_KERNEL)

    return mask


def find_largest_arrow_contour(mask):
    """
    Finds the largest arrow-coloured contour.
    Returns None if no large enough contour is found.
    """

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)

    if cv2.contourArea(largest) < MIN_AREA:
        return None

    return largest


def detect_arrow_direction(contour):
    """
    Detects whether the arrow points LEFT or RIGHT.
    Returns "LEFT", "RIGHT", or "NOT_ARROW".
    """

    if contour is None or len(contour) < 5:
        return "NOT_ARROW"

    # Bounding box is needed internally for object width and upper-region filtering
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

    # Only look for the tip in the upper part of the contour
    upper_limit = y + int(0.65 * h)
    upper_points = points[points[:, 1] < upper_limit]

    if len(upper_points) == 0:
        return "NOT_ARROW"

    # Find the farthest upper contour point from the centre
    distances = (upper_points[:, 0] - cx) ** 2 + (upper_points[:, 1] - cy) ** 2
    tip = upper_points[np.argmax(distances)]

    tip_x = int(tip[0])
    dx = tip_x - cx

    if abs(dx) < MIN_TIP_OFFSET_FRAC * w:
        return "NOT_ARROW"
    elif dx > 0:
        return "RIGHT"
    else:
        return "LEFT"


try:
    while True:
        frame = picam2.capture_array()

        # Calculate FPS
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        if dt > 0:
            fps = 1.0 / dt

        # RGB888 frame, so use RGB2HSV
        # Keeping this the same as your working line-following code
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # -----------------------------
        # Line following detection
        # -----------------------------

        # Threshold each line colour
        yellow_mask = mask_and_clean(hsv, YELLOW_LOWER, YELLOW_UPPER)
        blue_mask = mask_and_clean(hsv, BLUE_LOWER, BLUE_UPPER)

        # Find x position of each line at the lookahead row
        yellow_x = get_line_x(yellow_mask, LOOKAHEAD_Y, BAND_HEIGHT)
        blue_x = get_line_x(blue_mask, LOOKAHEAD_Y, BAND_HEIGHT)

        lane_centre_x = None

        if yellow_x is not None and blue_x is not None:
            # Both lines visible
            lane_centre_x = (yellow_x + blue_x) / 2

            # Save lane width for later in case one line disappears
            measured_width = blue_x - yellow_x
            if measured_width > 0:
                last_lane_width = measured_width

        elif yellow_x is not None and last_lane_width is not None:
            # Only yellow line visible
            lane_centre_x = yellow_x + last_lane_width / 2

        elif blue_x is not None and last_lane_width is not None:
            # Only blue line visible
            lane_centre_x = blue_x - last_lane_width / 2

        if lane_centre_x is not None:
            image_centre_x = FRAME_W / 2

            error_px = lane_centre_x - image_centre_x
            error = error_px / (FRAME_W / 2)

            # Simple smoothing
            error = 0.7 * last_error + 0.3 * error
            last_error = error

        else:
            # No lines found, keep last error
            error = last_error

        # -----------------------------
        # Arrow detection
        # -----------------------------

        arrow_mask = get_arrow_mask(hsv)
        arrow_contour = find_largest_arrow_contour(arrow_mask)
        arrow_direction = detect_arrow_direction(arrow_contour)

        arrow_visible = arrow_direction == "LEFT" or arrow_direction == "RIGHT"

        # Trigger the bias only on the first frame where the arrow appears.
        # This stops the timer from constantly restarting while the same arrow is visible.
        if arrow_visible and not arrow_visible_last and arrow_bias_start_time is None:
            arrow_bias_start_time = current_time
            arrow_bias_direction = arrow_direction
            print("Arrow triggered:", arrow_direction)

        arrow_visible_last = arrow_visible

        if arrow_direction != last_arrow_print_direction:
            print("Arrow direction:", arrow_direction)
            last_arrow_print_direction = arrow_direction

        # Work out the timed arrow bias
        arrow_bias = 0.0

        if arrow_bias_start_time is not None:
            arrow_elapsed = current_time - arrow_bias_start_time

            if arrow_elapsed < ARROW_STRONG_TIME:
                bias_strength = ARROW_STRONG_BIAS

            elif arrow_elapsed < ARROW_STRONG_TIME + ARROW_MODERATE_TIME:
                bias_strength = ARROW_MODERATE_BIAS

            elif arrow_elapsed < ARROW_STRONG_TIME + ARROW_MODERATE_TIME + ARROW_WEAK_TIME:
                bias_strength = ARROW_WEAK_BIAS

            else:
                bias_strength = 0.0
                arrow_bias_start_time = None
                arrow_bias_direction = None

            if arrow_bias_direction == "RIGHT":
                arrow_bias = ARROW_RIGHT_SIGN * bias_strength

            elif arrow_bias_direction == "LEFT":
                arrow_bias = -ARROW_RIGHT_SIGN * bias_strength

        # Add arrow bias to the normal line-following error
        final_error = error + arrow_bias

        # This is the value to send to the microcontroller
        error_int = int(final_error * 100)
        error_int = max(-100, min(100, error_int))  # Clamp to -100 to 100

        # Send to microcontroller over serial
        msg = f"D,{error_int},{base_speed}\n"
        ser.write(msg.encode("utf-8"))

        print(error_int)

        # -----------------------------
        # Debug display
        # -----------------------------

        # Convert RGB camera frame to BGR for OpenCV display
        display = frame.copy()

        # Create one combined mask image
        mask_display = np.zeros_like(display)

        # OpenCV display uses BGR colours:
        # Yellow = (0, 255, 255)
        # Blue   = (255, 0, 0)
        # Green  = (0, 255, 0)
        mask_display[yellow_mask > 0] = (0, 255, 255)
        mask_display[blue_mask > 0] = (255, 0, 0)
        mask_display[arrow_mask > 0] = (0, 255, 0)

        # Draw arrow contour only, no bounding box or points
        if arrow_contour is not None:
            cv2.drawContours(display, [arrow_contour], -1, (0, 255, 0), 2)

        # Draw lookahead line on both images
        cv2.line(display, (0, LOOKAHEAD_Y), (FRAME_W, LOOKAHEAD_Y), (255, 255, 255), 2)
        cv2.line(mask_display, (0, LOOKAHEAD_Y), (FRAME_W, LOOKAHEAD_Y), (255, 255, 255), 2)

        # Draw image centre
        cv2.line(display, (FRAME_W // 2, 0), (FRAME_W // 2, FRAME_H), (255, 255, 255), 2)

        if yellow_x is not None:
            cv2.circle(display, (yellow_x, LOOKAHEAD_Y), 8, (0, 255, 255), -1)

        if blue_x is not None:
            cv2.circle(display, (blue_x, LOOKAHEAD_Y), 8, (255, 0, 0), -1)

        if lane_centre_x is not None:
            cv2.circle(display, (int(lane_centre_x), LOOKAHEAD_Y), 8, (0, 255, 0), -1)

        cv2.putText(
            display,
            f"Error: {error_int}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )

        cv2.putText(
            display,
            f"Line error: {int(error * 100)}",
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.putText(
            display,
            f"Arrow: {arrow_direction}",
            (20, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0) if arrow_direction != "NOT_ARROW" else (0, 0, 255),
            2
        )

        cv2.putText(
            display,
            f"Bias: {int(arrow_bias * 100)}",
            (20, 160),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.putText(
            display,
            f"FPS: {fps:.1f}",
            (20, 200),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        # Put camera view and mask view side by side
        combined = np.hstack((display, mask_display))

        cv2.imshow("Lane Detection | Masks", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break

finally:
    msg = f"D,{0},{0}\n"
    ser.write(msg.encode("utf-8"))  # Send stop command to microcontroller
    ser.close()
    cv2.destroyAllWindows()
    picam2.stop()