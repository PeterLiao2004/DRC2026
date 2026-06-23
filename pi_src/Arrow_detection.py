import cv2
import numpy as np
import time
from picamera2 import Picamera2


# ============================================================
# USER SETTINGS
# ============================================================

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

# HSV threshold placeholders
# OpenCV HSV ranges:
# H: 0–179
# S: 0–255
# V: 0–255
ARROW_LOWER = np.array([11, 110, 63])
ARROW_UPPER = np.array([21, 173, 79])

# Minimum detected object size
MIN_AREA_FRAC = 0.005
MIN_AREA = int(FRAME_WIDTH * FRAME_HEIGHT * MIN_AREA_FRAC)

# Mask cleanup
KERNEL_SIZE = 7

# Arrow detection tuning
MIN_TIP_OFFSET_FRAC = 0.10

time.sleep(0.5)


# ============================================================
# PROCESSING FUNCTIONS
# ============================================================

def create_hsv_mask(frame_rgb):
    """
    Converts RGB camera frame to HSV and thresholds it.
    White pixels in the mask are detected object pixels.
    """

    hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
    

    return mask


def mask_and_clean(hsv, lower, upper, kernel_size=5):
    """
    Creates a binary mask for the given HSV range and applies morphological opening to clean it.
    """

    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    cleaned_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    return cleaned_mask


def find_largest_contour(mask):
    """
    Finds the largest valid contour in the mask.
    """

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if len(contours) == 0:
        return None

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < MIN_AREA:
        return None

    return largest


def detect_arrow_direction(contour):
    """
    Uses tip detection to classify the arrow.

    Returns:
        direction: "LEFT", "RIGHT", or "NOT_ARROW"
        debug: useful points for drawing
    """

    debug = {}

    if contour is None or len(contour) < 5:
        return "NOT_ARROW", debug

    area = cv2.contourArea(contour)

    if area < MIN_AREA:
        return "NOT_ARROW", debug

    x, y, w, h = cv2.boundingRect(contour)

    if w <= 0 or h <= 0:
        return "NOT_ARROW", debug

    # Reject very sparse weird shapes
    fill_ratio = area / float(w * h)

    if fill_ratio < 0.05:
        return "NOT_ARROW", debug

    # Find contour centre
    M = cv2.moments(contour)

    if M["m00"] == 0:
        return "NOT_ARROW", debug

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    points = contour[:, 0, :]

    # Since arrow base is at the bottom, look for the tip
    # mainly in the upper part of the object
    upper_limit = y + int(0.65 * h)
    upper_points = points[points[:, 1] < upper_limit]

    if len(upper_points) == 0:
        return "NOT_ARROW", debug

    # Farthest upper contour point from centre = likely arrow tip
    distances = (upper_points[:, 0] - cx) ** 2 + (upper_points[:, 1] - cy) ** 2
    tip = upper_points[np.argmax(distances)]

    tip_x = int(tip[0])
    tip_y = int(tip[1])

    dx = tip_x - cx
    min_required_offset = MIN_TIP_OFFSET_FRAC * w

    if abs(dx) < min_required_offset:
        direction = "NOT_ARROW"
    elif dx > 0:
        direction = "RIGHT"
    else:
        direction = "LEFT"

    confidence = min(abs(dx) / (w / 2), 1.0)

    debug = {
        "bbox": (x, y, w, h),
        "centre": (cx, cy),
        "tip": (tip_x, tip_y),
        "area": area,
        "fill_ratio": fill_ratio,
        "confidence": confidence
    }

    return direction, debug


def draw_overlay(frame_bgr, contour, direction, debug, fps):
    """
    Draws contour, bounding box, tip, centre, direction and FPS.
    """

    if contour is not None:
        cv2.drawContours(frame_bgr, [contour], -1, (0, 255, 0), 2)

    if debug:
        x, y, w, h = debug["bbox"]
        cx, cy = debug["centre"]
        tip_x, tip_y = debug["tip"]

        cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Centre
        cv2.circle(frame_bgr, (cx, cy), 6, (255, 0, 0), -1)

        # Tip
        cv2.circle(frame_bgr, (tip_x, tip_y), 8, (0, 0, 255), -1)

        # Centre-to-tip line
        cv2.line(frame_bgr, (cx, cy), (tip_x, tip_y), (0, 0, 255), 2)

        cv2.putText(
            frame_bgr,
            f"Confidence: {debug['confidence']:.2f}",
            (10, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )

    if direction == "NOT_ARROW":
        direction_colour = (0, 0, 255)
    else:
        direction_colour = (0, 255, 0)

    cv2.putText(
        frame_bgr,
        f"Direction: {direction}",
        (10, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        direction_colour,
        2
    )

    cv2.putText(
        frame_bgr,
        f"FPS: {fps:.1f}",
        (10, 115),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2
    )


# ============================================================
# MAIN LIVE LOOP
# ============================================================

prev_time = time.time()
last_direction = None

try:
    while True:
        # Capture live frame from Pi Camera 3
        frame_rgb = picam2.capture_array()

        # Mask processing
        hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
        mask_clean = mask_and_clean(hsv), ARROW_LOWER, ARROW_UPPER, KERNEL_SIZE)

        # Find largest object
        contour = find_largest_contour(mask_clean)

        # Detect arrow direction
        direction, debug = detect_arrow_direction(contour)

        # Only print when direction changes
        if direction != last_direction:
            print("Direction:", direction)
            last_direction = direction

        # Convert for OpenCV display
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        # FPS
        current_time = time.time()
        dt = current_time - prev_time
        fps = 1.0 / dt if dt > 0 else 0.0
        prev_time = current_time

        # Draw visual debugging
        draw_overlay(frame_bgr, contour, direction, debug, fps)

        # Resize only for screen display
        display_frame = cv2.resize(
            frame_bgr,
            (0, 0),
            fx=DISPLAY_SCALE,
            fy=DISPLAY_SCALE
        )

        display_mask = cv2.resize(
            mask_clean,
            (0, 0),
            fx=DISPLAY_SCALE,
            fy=DISPLAY_SCALE
        )

        cv2.imshow("Live Arrow Detection", display_frame)
        cv2.imshow("HSV Mask", display_mask)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q") or key == 27:
            break

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    cv2.destroyAllWindows()
    picam2.stop()
    print("Camera stopped.")