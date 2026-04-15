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
# Servo configuration
# ----------------------------
SERVO_CHANNELS = [0, 1, 2]
SERVO_MIN = 0
SERVO_MAX = 360
MAX_ANGLE = 90
SAFE_MARGIN = 0

current_angles = [45, 45, 45]

# ----------------------------
# Convert angle → PWM
# ----------------------------
def angle_to_pwm(angle):
    return int(SERVO_MIN + (angle / MAX_ANGLE) * (SERVO_MAX - SERVO_MIN))

# ----------------------------
# Move all servos together
# ----------------------------
def move_all_absolute(targets):
    global current_angles

    # Clamp targets
    targets = [
        max(SAFE_MARGIN, min(t, MAX_ANGLE - SAFE_MARGIN))
        for t in targets
    ]

    moving = True

    while moving:
        moving = False

        for i in range(3):
            current = current_angles[i]
            target = targets[i]

            if current != target:
                moving = True
                step = 1 if target > current else -1
                current_angles[i] += step

                pwm = angle_to_pwm(current_angles[i])
                pca.channels[SERVO_CHANNELS[i]].duty_cycle = pwm << 4

        time.sleep(0.02)

    print(f"[ABS ALL] → {current_angles}")

# ----------------------------
# Relative movement (all)
# ----------------------------
def move_all_relative(deltas):
    targets = [
        current_angles[i] + deltas[i]
        for i in range(3)
    ]
    move_all_absolute(targets)
    print(f"[REL ALL] by {deltas}")

# ----------------------------
# Main loop
# ----------------------------
try:
    print("3-Servo SYNC controller ready")
    print("Format:")
    print("  a x y z   (absolute)")
    print("  r dx dy dz (relative)")
    print("Example: a 30 45 60")

    while True:
        cmd = input("Enter command: ")

        if cmd.lower() == "q":
            break

        try:
            parts = cmd.split()
            mode = parts[0]
            values = list(map(int, parts[1:]))

            if len(values) != 3:
                raise ValueError

            if mode == "a":
                move_all_absolute(values)
            elif mode == "r":
                move_all_relative(values)
            else:
                print("Use 'a' or 'r'")

        except:
            print("Format: a 30 45 60 OR r 5 -5 10")

finally:
    for ch in SERVO_CHANNELS:
        pca.channels[ch].duty_cycle = 0

    pca.deinit()
    print("Cleaned up")