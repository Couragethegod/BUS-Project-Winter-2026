import cv2
import numpy as np
import time
from picamera2 import Picamera2
from adafruit_servokit import ServoKit

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

# ------------------ HSV RANGE FOR YELLOW ------------------
lower_yellow = np.array([20, 100, 100])  # Lower bound of yellow in HSV
upper_yellow = np.array([30, 255, 255])  # Upper bound of yellow in HSV

# ------------------ MAIN LOOP ------------------
while True:
    frame = picam2.capture_array()

    # Debug: Print frame shape and check format
    print("Frame shape:", frame.shape)  # (height, width, channels)

    # Check if the frame is in BGR format (expected format)
    if frame.shape[2] == 3:
        print("Frame is in BGR format.")  # This is expected for standard color images
    else:
        print("Frame format is incorrect, possible issues with camera settings.")

    # Convert the frame from BGR to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Debug: Show the HSV image directly (to visualize color space)
    cv2.imshow("HSV Image", hsv)

    # ------------------ MASK FOR YELLOW BALL ------------------
    # Use the lower and upper yellow bounds to create a binary mask
    mask_combined = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Optional: Clean the mask (remove small noise)
    mask_combined = cv2.GaussianBlur(mask_combined, (3, 3), 0)

    # ------------------ FIND CENTROID OF YELLOW AREA ------------------
    # Find all non-zero pixels in the mask
    yellow_pixels = np.where(mask_combined > 0)

    if len(yellow_pixels[0]) > 0:  # If we found yellow pixels
        # Calculate the centroid of the yellow pixels
        centroid_x = int(np.mean(yellow_pixels[1]))  # X coordinates (column index)
        centroid_y = int(np.mean(yellow_pixels[0]))  # Y coordinates (row index)

        # Draw a circle at the centroid of the detected yellow area
        cv2.circle(frame, (centroid_x, centroid_y), 5, (0, 255, 0), -1)  # Green dot

        # ------------------ PID CONTROL ------------------
        err_x = centroid_x - TARGET_X  # Error in X direction
        err_y = centroid_y - TARGET_Y  # Error in Y direction

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