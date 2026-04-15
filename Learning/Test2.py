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
# SAFE LIMITS (MG996R)
# ----------------------------
SERVO_MIN = 320
SERVO_MAX = 420
MAX_ANGLE = 90
SAFE_MARGIN = 5

# ----------------------------
# Position tracking
# ----------------------------
current_angle = 45  # safe startup middle position

# ----------------------------
# Convert angle to PWM
# ----------------------------
def angle_to_pwm(angle):
    return int(SERVO_MIN + (angle / MAX_ANGLE) * (SERVO_MAX - SERVO_MIN))

# ----------------------------
# LOW-LEVEL move (absolute)
# ----------------------------
def move_absolute(angle):
    global current_angle

    # safety clamp
    angle = max(SAFE_MARGIN, min(angle, MAX_ANGLE - SAFE_MARGIN))

    step = 1 if angle > current_angle else -1

    for a in range(current_angle, angle, step):
        pca.channels[0].duty_cycle = angle_to_pwm(a) << 4
        time.sleep(0.02)

    current_angle = angle
    print(f"[ABS] Moved to {current_angle}°")

# ----------------------------
# HIGH-LEVEL move (relative)
# ----------------------------
def move_relative(delta):
    target = current_angle + delta
    move_absolute(target)
    print(f"[REL] Moved by {delta}°")

# ----------------------------
# Main loop
# ----------------------------
try:
    print("Hybrid servo control ready")
    move_absolute(45)  # safe startup position
    time.sleep(0.5)

    while True:
        cmd = input("Enter (a=absolute, r=relative, q=quit): ")

        if cmd == "q":
            break

        try:
            mode, value = cmd.split()
            value = int(value)

            if mode == "a":
                move_absolute(value)
            elif mode == "r":
                move_relative(value)
            else:
                print("Use: a 30 or r 5")
        except:
            print("Format: a 30 (absolute) OR r 5 (relative)")

finally:
    pca.channels[0].duty_cycle = 0
    pca.deinit()
    print("Cleaned up")