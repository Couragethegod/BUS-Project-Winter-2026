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
# SAFE LIMITS (tune if needed)
# ----------------------------
SERVO_MIN = 320
SERVO_MAX = 420
MAX_ANGLE = 90
SAFE_MARGIN = 5

# ----------------------------
# Software position tracking
# (NO physical movement on startup)
# ----------------------------
current_angle = 45  # only a software assumption

# ----------------------------
# Convert angle → PWM
# ----------------------------
def angle_to_pwm(angle):
    return int(SERVO_MIN + (angle / MAX_ANGLE) * (SERVO_MAX - SERVO_MIN))

# ----------------------------
# Absolute movement
# ----------------------------
def move_absolute(target):
    global current_angle

    # safety clamp
    target = max(SAFE_MARGIN, min(target, MAX_ANGLE - SAFE_MARGIN))

    step = 1 if target > current_angle else -1

    for a in range(current_angle, target, step):
        pca.channels[0].duty_cycle = angle_to_pwm(a) << 4
        time.sleep(0.02)

    current_angle = target
    print(f"[ABS] Moved to {current_angle}°")

# ----------------------------
# Relative movement
# ----------------------------
def move_relative(delta):
    target = current_angle + delta
    move_absolute(target)
    print(f"[REL] Moved by {delta}°")

# ----------------------------
# Main loop (NO startup motion)
# ----------------------------
try:
    print("Servo controller ready (absolute + relative)")
    print("No automatic movement on startup")

    while True:
        cmd = input("Enter (a=absolute, r=relative, q=quit): ")

        if cmd.lower() == "q":
            break

        try:
            mode, value = cmd.split()
            value = int(value)

            if mode == "a":
                move_absolute(value)
            elif mode == "r":
                move_relative(value)
            else:
                print("Use: a 30 OR r 5")
        except:
            print("Format: a 30 OR r 5")

finally:
    # stop PWM output
    pca.channels[0].duty_cycle = 0
    pca.deinit()
    print("Cleaned up")