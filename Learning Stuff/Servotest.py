import time
import board
import busio
from adafruit_pca9685 import PCA9685

# Initialize I2C and PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)

# Set frequency FIRST
pca.frequency = 50

# Servo channels (0, 1, 2)
servo_channels = [0]

# Store angles
servo_angles = [90]
offsets = [0]

# Pulse range (adjust if needed)
SERVO_MIN = 150
SERVO_MAX = 600

def angle_to_pwm(angle):
    return int(SERVO_MIN + (angle / 180.0) * (SERVO_MAX - SERVO_MIN))


# 🛑 IMPORTANT: disable all servo outputs at startup
def disable_all_servos():
    for ch in servo_channels:
        pca.channels[ch].duty_cycle = 0

disable_all_servos()


def set_servo(index, relative_angle):
    absolute_angle = offsets[index] + relative_angle

    if 0 <= absolute_angle <= 180:
        pwm_value = angle_to_pwm(absolute_angle)

        # ✅ Only NOW we send signal to servo
        pca.channels[servo_channels[index]].duty_cycle = pwm_value << 4

        servo_angles[index] = absolute_angle

        print(f"Servo {index+1}: {relative_angle}° (relative), {absolute_angle}° (absolute)")
    else:
        print("⚠️ Angle out of range (0–180 after offset)")


def calibrate_zero():
    global offsets
    offsets = servo_angles.copy()
    print("✅ Current positions set as new 0°")


def show_angles():
    print("\n=== Servo Angles ===")
    for i in range(len(servo_channels)):
        relative = servo_angles[i] - offsets[i]
        print(f"Servo {i+1}: {relative}° (relative) | {servo_angles[i]}° (absolute)")
    print("====================\n")


# Main loop
try:
    print("System ready. Servos disabled until input is given.")

    while True:
        show_angles()

        cmd = input("Enter: servo, angle | 'z' = zero | 'q' = quit: ")

        if cmd.lower() == 'q':
            break
        elif cmd.lower() == 'z':
            calibrate_zero()
            continue

        try:
            s, a = map(int, cmd.split())
            set_servo(s - 1, a)
        except:
            print("Invalid input. Example: 1 45")

except KeyboardInterrupt:
    pass

finally:
    # 🔒 shut down cleanly
    for ch in servo_channels:
        pca.channels[ch].duty_cycle = 0

    pca.deinit()
    print("Cleaned up.")