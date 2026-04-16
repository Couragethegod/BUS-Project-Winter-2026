import time
import cv2
import numpy as np

from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# ---------------------------
# CAMERA SETTINGS (FIXED)
# ---------------------------
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

cx = FRAME_WIDTH // 2
cy = FRAME_HEIGHT // 2

# ---------------------------
# PCA9685 SETUP
# ---------------------------
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

SERVO_X = 0
SERVO_Y = 1

MIN_ANGLE = 45
MAX_ANGLE = 80

servo_x = 62
servo_y = 62


def angle_to_duty(angle):
    pulse_min = 500
    pulse_max = 2500
    pulse = pulse_min + (angle / 180.0) * (pulse_max - pulse_min)
    return int(pulse * 65535 / 20000)


def set_servo(channel, angle):
    angle = max(MIN_ANGLE, min(MAX_ANGLE, angle))
    pca.channels[channel].duty_cycle = angle_to_duty(angle)


set_servo(SERVO_X, servo_x)
set_servo(SERVO_Y, servo_y)

time.sleep(1)

# ---------------------------
# CAMERA INIT
# ---------------------------
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

if not cap.isOpened():
    print("Camera failed")
    exit()

# ---------------------------
# YELLOW DETECTION
# ---------------------------
lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([35, 255, 255])

# ---------------------------
# PID CONSTANTS
# ---------------------------
Kp = 0.015
Ki = 0.0005
Kd = 0.008

integral_x = 0
integral_y = 0

prev_error_x = 0
prev_error_y = 0

prev_time = time.time()

INTEGRAL_LIMIT = 1000

# ---------------------------
# LOOP
# ---------------------------
while True:
    ret, frame = cap.read()

    if not ret or frame is None:
        continue

    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c = max(contours, key=cv2.contourArea)

        if cv2.contourArea(c) > 300:
            (x, y), radius = cv2.minEnclosingCircle(c)

            error_x = x - cx
            error_y = y - cy

            # PID X
            integral_x += error_x * dt
            integral_x = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, integral_x))

            derivative_x = (error_x - prev_error_x) / dt if dt > 0 else 0

            output_x = Kp * error_x + Ki * integral_x + Kd * derivative_x
            prev_error_x = error_x

            # PID Y
            integral_y += error_y * dt
            integral_y = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, integral_y))

            derivative_y = (error_y - prev_error_y) / dt if dt > 0 else 0

            output_y = Kp * error_y + Ki * integral_y + Kd * derivative_y
            prev_error_y = error_y

            # Apply
            servo_x -= output_x
            servo_y += output_y

            # Clamp
            servo_x = max(MIN_ANGLE, min(MAX_ANGLE, servo_x))
            servo_y = max(MIN_ANGLE, min(MAX_ANGLE, servo_y))

            set_servo(SERVO_X, servo_x)
            set_servo(SERVO_Y, servo_y)

            # Debug visuals
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

    cv2.imshow("PID Balance", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
pca.deinit()