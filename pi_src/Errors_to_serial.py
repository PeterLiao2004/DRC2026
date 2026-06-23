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

# -----------------------------
# Line detection settings
# -----------------------------

LOOKAHEAD_Y = int(FRAME_H * 0.55)   # where to look for the lines
BAND_HEIGHT = 80                    # thickness of horizontal band
MIN_PIXELS = 50                     # minimum pixels needed to trust a line

last_error = 0.0
last_lane_width = None

base_speed = 30  # Base speed to send to microcontroller (0-100)

# -----------------------------
# FPS settings
# -----------------------------
prev_time = time.time()
fps = 0.0


def get_line_x(mask, y, band_height):
    """
    Finds the average x position of a line inside a horizontal band.
    Returns None if not enough pixels are found.
    """

    y1 = max(0, y - band_height // 2)
    y2 = min(mask.shape[0], y + band_height // 2)

    band = mask[y1:y2, :]
    ys, xs = np.where(band > 0)

    if len(xs) < MIN_PIXELS:
        return None

    return int(np.median(xs))



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
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Threshold each line colour
        yellow_mask = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)
        blue_mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)

        # Clean up small noise
        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)

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

        # This is the value to send to the microcontroller
        error_int = int(error * 100)

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
        mask_display[yellow_mask > 0] = (0, 255, 255)
        mask_display[blue_mask > 0] = (255, 0, 0)

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
            f"FPS: {fps:.1f}",
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
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
    cv2.destroyAllWindows()
    picam2.stop()