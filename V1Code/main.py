import cv2
import numpy as np
import time
from picamera2 import Picamera2
from adafruit_servokit import ServoKit
from collections import deque

# ------------------ SERVO SETUP ------------------
kit = ServoKit(channels=16)

SERVO_MIN = 45
SERVO_MAX = 80
SERVO_NEUTRAL = 62  # Neutral position, adjust to your platform level

def clamp(val):
    """Ensure servo values stay within the valid range."""
    return max(SERVO_MIN, min(SERVO_MAX, val))

def set_servos(a1, a2, a3):
    """Set servo angles."""
    kit.servo[0].angle = clamp(a1)
    kit.servo[1].angle = clamp(a2)
    kit.servo[2].angle = clamp(a3)

# ------------------ CAMERA SETUP ------------------
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (320, 240)}))
picam2.start()

# ------------------ PID PARAMETERS ------------------
kp = 0.05
kd = 0.02
prev_err_x = 0
prev_err_y = 0

# ------------------ TARGET COORDINATES ------------------
TARGET_X = 160  # Center X (horizontal center of the frame)
TARGET_Y = 120  # Center Y (vertical center of the frame)

# ------------------ YELLOW COLOR DETECTION RANGE ------------------
lower_yellow = np.array([20, 100, 100])  # Lower bound of yellow in HSV
upper_yellow = np.array([30, 255, 255])  # Upper bound of yellow in HSV

# ------------------ MAIN LOOP ------------------
while True:
    frame = picam2.capture_array()

    # Ensure we're working with the correct color format
    print("Frame shape:", frame.shape)  # Debug to check frame format and dimensions

    # Convert the frame from BGR to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # ------------------ MASK FOR YELLOW BALL ------------------
    # Apply Gaussian blur to reduce noise
    frame_blurred = cv2.GaussianBlur(hsv, (5, 5), 0)

    # Create the HSV mask to detect yellow color
    mask_hsv = cv2.inRange(frame_blurred, lower_yellow, upper_yellow)

    # Convert the frame to grayscale to combine the detection
    frame_gray = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2GRAY)
    gray_threshold = 60
    mask_gray = cv2.threshold(frame_gray, gray_threshold, 255, cv2.THRESH_BINARY_INV)[1]

    # Combine the HSV and grayscale masks
    mask_combined = cv2.bitwise_or(mask_hsv, mask_gray)

    # Process the combined mask (erosion and dilation to remove noise)
    mask_eroded = cv2.erode(mask_combined, None, iterations=1)
    mask_dilated = cv2.dilate(mask_eroded, None, iterations=1)

    # ------------------ FIND CONTOURS (BALL DETECTION) ------------------
    contours, _ = cv2.findContours(mask_dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the largest contour by area (most likely the yellow ball)
        c = max(contours, key=cv2.contourArea)

        # Get the minimum enclosing circle for the contour
        (x, y), r = cv2.minEnclosingCircle(c)
        r = int(r)

        if r > 0.5:  # Only consider if the radius is large enough to be a valid detection
            x, y = int(x), int(y)

            # Draw the circle around the detected yellow ball
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)  # Green circle for ball

            # ------------------ PID CONTROL ------------------
            err_x = x - TARGET_X  # Error in X direction
            err_y = y - TARGET_Y  # Error in Y direction

            # Derivative of error
            d_err_x = err_x - prev_err_x
            d_err_y = err_y - prev_err_y

            # PID control calculation for X and Y movements
            control_x = kp * err_x + kd * d_err_x
            control_y = kp * err_y + kd * d_err_y

            # Update previous errors
            prev_err_x = err_x
            prev_err_y = err_y

            # ------------------ SERVO MIXING ------------------
            s1 = SERVO_NEUTRAL + control_x
            s2 = SERVO_NEUTRAL - control_x / 2 + control_y
            s3 = SERVO_NEUTRAL - control_x / 2 - control_y

            # Set servo positions
            set_servos(s1, s2, s3)

            # Debug: Draw target center on the frame
            cv2.circle(frame, (TARGET_X, TARGET_Y), 5, (0, 0, 255), -1)  # Red dot for target center

    # ------------------ DISPLAY ------------------
    # Show the frame with detected yellow ball and target
    cv2.imshow("Ball Balancer", frame)

    # Exit if the user presses 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up and close windows
cv2.destroyAllWindows()
picam2.close()