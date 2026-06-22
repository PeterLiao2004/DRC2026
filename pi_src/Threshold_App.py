import cv2
import numpy as np
import picamera2
from picamera2 import Picamera2

def nothing(x):
    pass

# Initialize camera
picam2 = Picamera2()

frameSizeX = 820  # 820
frameSizeY = 616  # 616
minArea = int(frameSizeX * frameSizeY / 400)
currentFrame = np.zeros((frameSizeY, frameSizeX, 3), np.uint8)

config = picam2.create_video_configuration(
    main={"format": 'RGB888', "size": (frameSizeX, frameSizeY)},
    controls={'FrameRate': 50},
    raw={'size': (1640, 1232)}
)

picam2.configure(config)
picam2.set_controls({"ExposureTime": 8000, "AnalogueGain": 2.0, "ColourGains": (2.1, 1.6)})
picam2.start()

# Create trackbars for HSV range adjustment
cv2.namedWindow('Thresholder_App')
cv2.createTrackbar("VMax", "Thresholder_App", 0, 255, nothing)
cv2.createTrackbar("VMin", "Thresholder_App", 0, 255, nothing)
cv2.createTrackbar("SMax", "Thresholder_App", 0, 255, nothing)
cv2.createTrackbar("SMin", "Thresholder_App", 0, 255, nothing)
cv2.createTrackbar("HMax", "Thresholder_App", 0, 179, nothing)
cv2.createTrackbar("HMin", "Thresholder_App", 0, 179, nothing)

# Set default positions for trackbars
cv2.setTrackbarPos("VMax", "Thresholder_App", 255)
cv2.setTrackbarPos("VMin", "Thresholder_App", 0)
cv2.setTrackbarPos("SMax", "Thresholder_App", 255)
cv2.setTrackbarPos("SMin", "Thresholder_App", 0)
cv2.setTrackbarPos("HMax", "Thresholder_App", 179)
cv2.setTrackbarPos("HMin", "Thresholder_App", 0)

while True:
    # Capture frame from the camera
    frame = picam2.capture_array()

    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Get trackbar positions for HSV values
    vmax = cv2.getTrackbarPos("VMax", "Thresholder_App")
    vmin = cv2.getTrackbarPos("VMin", "Thresholder_App")
    smax = cv2.getTrackbarPos("SMax", "Thresholder_App")
    smin = cv2.getTrackbarPos("SMin", "Thresholder_App")
    hmax = cv2.getTrackbarPos("HMax", "Thresholder_App")
    hmin = cv2.getTrackbarPos("HMin", "Thresholder_App")

    # Create HSV range mask
    min_ = np.array([hmin, smin, vmin])
    max_ = np.array([hmax, smax, vmax])
    mask = cv2.inRange(hsv, min_, max_)

    # Apply the mask to the frame
    thresholded_img = cv2.bitwise_and(frame, frame, mask=mask)

    # Show the thresholded image
    cv2.imshow("Thresholder_App", thresholded_img)

    # Wait for key press; exit if 'q' or 'esc' is pressed
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q') or k == 27:
        break

# Print final HSV values
print("Lower:", [hmin, smin, vmin])
print("Upper:", [hmax, smax, vmax])


# Release resources
cv2.destroyAllWindows()
picam2.stop()