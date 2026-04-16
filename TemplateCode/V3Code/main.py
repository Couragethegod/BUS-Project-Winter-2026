import cv2
import numpy as np
import time
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# =========================
# PCA9685 SETUP
# =========================
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

SERVO_CHANNELS = [0, 1, 2]

# Servo limits (degrees)
MIN_ANGLE = 45
MAX_ANGLE = 80
NEUTRAL = 62  # middle point

# Convert angle → PWM
def angle_to_pwm(angle):
    pulse_min = 150  # adjust if needed
    pulse_max = 600
    return int(pulse_min + (angle / 180.0) * (pulse_max - pulse_min))

def set_servo(channel, angle):
    angle = max(MIN_ANGLE, min(MAX_ANGLE, angle))
    pwm = angle_to_pwm(angle)
    pca.channels[channel].duty_cycle = pwm

# =========================
# PID CONTROLLER
# =========================
Kp = 0.08
Ki = 0.0005
Kd = 0.04

prev_error_x = 0
prev_error_y = 0
integral_x = 0
integral_y = 0

# =========================
# CAMERA SETUP
# =========================
cap = cv2.VideoCapture(0)

# =========================
# TEAL COLOR RANGE (HSV)
# =========================
lower_teal = np.array([80, 100, 100])
upper_teal = np.array([100, 255, 255])

# =========================
# SERVO GEOMETRY (120 deg)
# =========================
angles_deg = [0, 120, 240]
angles_rad = [np.radians(a) for a in angles_deg]

# =========================
# MAIN LOOP
# =========================
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    frame = cv2.flip(frame, -1)
    h, w, _ = frame.shape
    center_x, center_y = w // 2, h // 2

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_teal, upper_teal)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 500:

            (x, y), radius = cv2.minEnclosingCircle(largest)
            x, y = int(x), int(y)

            # Draw ball
            cv2.circle(frame, (x, y), int(radius), (0, 255, 0), 2)

            # ERROR
            error_x = x - center_x
            error_y = y - center_y

            # PID
            integral_x += error_x
            integral_y += error_y

            derivative_x = error_x - prev_error_x
            derivative_y = error_y - prev_error_y

            output_x = Kp * error_x + Ki * integral_x + Kd * derivative_x
            output_y = Kp * error_y + Ki * integral_y + Kd * derivative_y

            prev_error_x = error_x
            prev_error_y = error_y

            # =========================
            # MAP TO 3 SERVOS
            # =========================
            for i, angle in enumerate(angles_rad):
                contribution = output_x * np.cos(angle) + output_y * np.sin(angle)
                servo_angle = NEUTRAL + contribution * 0.05
                set_servo(SERVO_CHANNELS[i], servo_angle)

    # Draw center
    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

    cv2.imshow("Ball Balancer", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
pca.deinit()