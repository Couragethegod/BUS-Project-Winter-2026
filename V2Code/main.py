import cv2
import numpy as np
import time
from picamera2 import Picamera2
from adafruit_servokit import ServoKit

# ------------------ SERVO SETUP ------------------
kit = ServoKit(channels=16)

SERVO_IDS = [2, 0, 1]  # change if needed
SERVO_MIN = 45
SERVO_MAX = 80
SERVO_NEUTRAL = 60

def clamp(val):
    return max(SERVO_MIN, min(SERVO_MAX, val))

# Initialize servos to neutral
for s in SERVO_IDS:
    kit.servo[s].angle = SERVO_NEUTRAL

# ------------------ PID PARAMETERS ------------------
Kp = 0.035
Ki = 0.000
Kd = 0.01

integral_x = 0
integral_y = 0
prev_error_x = 0
prev_error_y = 0

# ------------------ CAMERA SETUP ------------------
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

time.sleep(2)

# ------------------ MAIN LOOP ------------------
while True:
    frame = picam2.capture_array()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Yellow ball mask (you may need to tune this)
    lower_yellow = np.array([90, 100, 100])
    upper_yellow = np.array([110, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    h, w = frame.shape[:2]
    center_x, center_y = w // 2, h // 2

    if contours:
        largest = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest) > 200:
            (x, y), radius = cv2.minEnclosingCircle(largest)
            ball_x, ball_y = int(x), int(y)

            # Draw
            cv2.circle(frame, (ball_x, ball_y), int(radius), (0,255,0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0,0,255), -1)

            # ------------------ PID ------------------
            error_x = center_x - ball_x
            error_y = center_y - ball_y

            integral_x += error_x
            integral_y += error_y

            derivative_x = error_x - prev_error_x
            derivative_y = error_y - prev_error_y

            output_x = Kp*error_x + Ki*integral_x + Kd*derivative_x
            output_y = Kp*error_y + Ki*integral_y + Kd*derivative_y

            prev_error_x = error_x
            prev_error_y = error_y

            # ------------------ 3 SERVO MAPPING ------------------
            # 120° spacing approximation
            s0 = SERVO_NEUTRAL + output_x
            s1 = SERVO_NEUTRAL - 0.5*output_x + 0.866*output_y
            s2 = SERVO_NEUTRAL - 0.5*output_x - 0.866*output_y

            angles = [s0, s1, s2]

            # Apply safely
            for i, s in enumerate(SERVO_IDS):
                kit.servo[s].angle = clamp(angles[i])

    # Show camera output
    cv2.imshow("Ball Tracker", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

# Cleanup
cv2.destroyAllWindows()
picam2.stop()