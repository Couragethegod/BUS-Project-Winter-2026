import time
import board
import busio
from adafruit_pca9685 import PCA9685

# ----------------------------
# Setup PCA9685
# ----------------------------
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# ----------------------------
# SAFE CALIBRATION (IMPORTANT)
# ----------------------------
SERVO_MIN = 320
SERVO_MAX = 400
MAX_ANGLE = 90
SAFE_MARGIN = 5

# ----------------------------
# Convert angle to PWM
# ----------------------------
def angle_to_pwm(angle):
    return int(SERVO_MIN + (angle / MAX_ANGLE) * (SERVO_MAX - SERVO_MIN))

# ----------------------------
# Smooth & safe movement
# ----------------------------
current_angle = 45  # start in a safe middle position

def move_servo(target):
    global current_angle

    # Clamp input to safe range
    if target < SAFE_MARGIN:
        target = SAFE_MARGIN
    if target > (MAX_ANGLE - SAFE_MARGIN):
        target = MAX_ANGLE - SAFE_MARGIN

    # Move gradually (prevents gear stress)
    step = 1 if target > current_angle else -1

    for angle in range(current_angle, target, step):
        pwm = angle_to_pwm(angle)
        pca.channels[0].duty_cycle = pwm << 4
        time.sleep(0.02)

    current_angle = target
    print(f"Moved safely to {current_angle}°")

# ----------------------------
# Main loop
# ----------------------------
try:
    print("Safe servo controller ready (0–90° range)")
    move_servo(45)  # center on startup
    time.sleep(0.5)

    while True:
        val = input("Enter angle (0–90) or q: ")

        if val.lower() == 'q':
            break

        try:
            angle = int(val)
            move_servo(angle)
        except:
            print("Enter a valid number")

finally:
    # Safe shutdown
    pca.channels[0].duty_cycle = 0
    pca.deinit()
    print("Cleaned up safely")