import cv2
import numpy as np
import time
from picamera2 import Picamera2
from adafruit_servokit import ServoKit

# ------------------ SERVO SETUP ------------------
kit = ServoKit(channels=16)

SERVO_MIN = 45
SERVO_MAX = 80
SERVO_NEUTRAL = 62  # adjust this to your platform level

def clamp(val):
    return max(SERVO_MIN, min(SERVO_MAX, val))

def set_servos(a1, a2, a3):
    kit.servo[0].angle = clamp(a1)
    kit.servo[1].angle = clamp(a2)
    kit.servo[2].angle = clamp(a3)

# ------------------ CAMERA SETUP ------------------
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (320, 240)}))
picam2.start()

# ------------------ PID ------------------
kp = 0.05
kd = 0.02
prev_err_x = 0
prev_err_y = 0

# ------------------ TARGET ------------------
TARGET_X = 160
TARGET_Y = 120

# ------------------ MAIN LOOP ------------------
while True:
    frame = picam2.capture_array()

    # Make sure we're working with the correct color format
    print("Frame shape:", frame.shape)  # Debug to check frame format and dimensions

    # Check if the frame is in BGR format
    if frame.shape[2] == 3:
        print("Frame is in BGR format.")  # This is expected for standard color images
    else:
        print("Frame format is incorrect, possible issues with camera settings.")

    # Convert to HSV (check for any issues with color conversion)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Debug: Show the HSV image directly (so we can see the conversion result)
    cv2.imshow("HSV Image", hsv)

    # Detect yellow ball (corrected HSV range for yellow)
    lower_yellow = np.array([20, 100, 100])  # Lower bound of yellow in HSV
    upper_yellow = np.array([30, 255, 255])  # Upper bound of yellow in HSV
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Clean noise (optional: smoothing to reduce small noise)
    mask = cv2.GaussianBlur(mask, (5, 5), 0)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c = max(contours, key=cv2.contourArea)
        (x, y), r = cv2.minEnclosingCircle(c)

        if r > 5:
            x, y = int(x), int(y)

            # Draw the circle around the detected yellow ball
            cv2.circle(frame, (x, y), int(r), (0, 255, 0), 2)  # Green circle for debugging

            # -------- PID --------
            err_x = x - TARGET_X
            err_y = y - TARGET_Y

            d_err_x = err_x - prev_err_x
            d_err_y = err_y - prev_err_y

            control_x = kp * err_x + kd * d_err_x
            control_y = kp * err_y + kd * d_err_y

            prev_err_x = err_x
            prev_err_y = err_y

            # -------- SERVO MIXING --------
            s1 = SERVO_NEUTRAL + control_x
            s2 = SERVO_NEUTRAL - control_x / 2 + control_y
            s3 = SERVO_NEUTRAL - control_x / 2 - control_y

            set_servos(s1, s2, s3)

            # Debug: Draw target center
            cv2.circle(frame, (TARGET_X, TARGET_Y), 5, (0, 0, 255), -1)  # Red dot for target
           
    # Show the frame
    cv2.imshow("Ball Balancer", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.close()