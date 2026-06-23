import cv2
import numpy as np
import time
from picamera2 import Picamera2


# ============================================================
# SETTINGS
# ============================================================

FRAME_W = 960
FRAME_H = 540
FPS_TARGET = 40

# Your tuned HSV thresholds
ARROW_LOWER = np.array([10, 97, 15])
ARROW_UPPER = np.array([31, 172, 120])

MIN_AREA_FRAC = 0.005
MIN_AREA = int(FRAME_W * FRAME_H * MIN_AREA_FRAC)

KERNEL_SIZE = 3
KERNEL = np.ones((KERNEL_SIZE, KERNEL_SIZE), np.uint8)

MIN_TIP_OFFSET_FRAC = 0.10


# ============================================================
# CAMERA SETUP
# ============================================================

picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={"format": "RGB888", "size": (FRAME_W, FRAME_H)},
    controls={"FrameRate": FPS_TARGET}
)

picam2.configure(config)
picam2.start()

picam2.set_controls({
    "AeEnable": False,
    "AwbEnable": False,
    "ExposureTime": 13000,
    "AnalogueGain": 2.8,
    "ColourGains": (2.1, 1.6)
})

time.sleep(0.5)


# ============================================================
# FUNCTIONS
# ============================================================

def get_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, ARROW_LOWER, ARROW_UPPER)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)

    return mask


def find_largest_contour(mask):
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
    if contour is None or len(contour) < 5:
        return "NOT_ARROW", None, None, None

    x, y, w, h = cv2.boundingRect(contour)

    if w <= 0 or h <= 0:
        return "NOT_ARROW", None, None, None

    M = cv2.moments(contour)

    if M["m00"] == 0:
        return "NOT_ARROW", None, None, None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    points = contour[:, 0, :]

    # Look for the arrow tip in the upper part of the object
    upper_limit = y + int(0.65 * h)
    upper_points = points[points[:, 1] < upper_limit]

    if len(upper_points) == 0:
        return "NOT_ARROW", (cx, cy), None, (x, y, w, h)

    distances = (upper_points[:, 0] - cx) ** 2 + (upper_points[:, 1] - cy) ** 2
    tip = upper_points[np.argmax(distances)]

    tip_x = int(tip[0])
    tip_y = int(tip[1])

    dx = tip_x - cx

    if abs(dx) < MIN_TIP_OFFSET_FRAC * w:
        direction = "NOT_ARROW"
    elif dx > 0:
        direction = "RIGHT"
    else:
        direction = "LEFT"

    return direction, (cx, cy), (tip_x, tip_y), (x, y, w, h)


def draw_output(frame, contour, direction, centre, tip, bbox, fps):
    if contour is not None:
        cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)

    if bbox is not None:
        x, y, w, h = bbox
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

    if centre is not None:
        cv2.circle(frame, centre, 5, (255, 0, 0), -1)

    if tip is not None:
        cv2.circle(frame, tip, 7, (0, 0, 255), -1)
        cv2.line(frame, centre, tip, (0, 0, 255), 2)

    if direction == "NOT_ARROW":
        colour = (0, 0, 255)
    else:
        colour = (0, 255, 0)

    cv2.putText(
        frame,
        f"Direction: {direction}",
        (10, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        colour,
        2
    )

    cv2.putText(
        frame,
        f"FPS: {fps:.1f}",
        (10, 75),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2
    )


# ============================================================
# MAIN LOOP
# ============================================================

prev_time = time.time()
last_direction = None

try:
    while True:
        frame = picam2.capture_array()

        mask = get_mask(frame)

        contour = find_largest_contour(mask)

        direction, centre, tip, bbox = detect_arrow_direction(contour)

        if direction != last_direction:
            print("Direction:", direction)
            last_direction = direction

        current_time = time.time()
        dt = current_time - prev_time
        fps = 1.0 / dt if dt > 0 else 0.0
        prev_time = current_time

        draw_output(frame, contour, direction, centre, tip, bbox, fps)

        cv2.imshow("Mask", mask)
        cv2.imshow("Arrow Detection", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q") or key == 27:
            break

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    cv2.destroyAllWindows()
    picam2.stop()
    print("Camera stopped.")