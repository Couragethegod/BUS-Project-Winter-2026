import cv2
import numpy as np
import time
from picamera2 import Picamera2
from adafruit_servokit import ServoKit

# ------------------ SERVO SETUP ------------------
kit = ServoKit(channels=16)

SERVO_MIN = 45
SERVO_MAX = 80
SERVO_NEUTRAL = 60

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
kp = 0.055
kd = 0.01
prev_err_x = 0
prev_err_y = 0

# ------------------ TARGET ------------------
TARGET_X = 160
TARGET_Y = 120

# ------------------ MAIN LOOP ------------------
while True:
    frame = picam2.capture_array()

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # -------- Teal DETECTION --------
    lower_teal = np.array([90, 100, 100])
    upper_teal = np.array([110, 255, 255])

    mask = cv2.inRange(hsv, lower_teal, upper_teal)

    # Clean noise
    mask = cv2.GaussianBlur(mask, (3, 3), 0)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=2)

    cv2.imshow("Mask", mask)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c = max(contours, key=cv2.contourArea)
        (x, y), r = cv2.minEnclosingCircle(c)

        if r > 5:
            x, y = int(x), int(y)

            # Draw
            cv2.circle(frame, (x, y), int(r), (0, 255, 0), 2)

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
            # Adjust signs depending on your robot orientation
            GAIN = 3.0 # increase for more aggressive response

            s1 = SERVO_NEUTRAL + control_x * GAIN
            s2 = SERVO_NEUTRAL - control_x/2 * GAIN + control_y * GAIN
            s3 = SERVO_NEUTRAL - control_x/2 * GAIN - control_y * GAIN

            set_servos(s1, s2, s3)

            # Debug
            cv2.circle(frame, (TARGET_X, TARGET_Y), 5, (0, 0, 255), -1)

    cv2.imshow("Ball Balancer", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.close()