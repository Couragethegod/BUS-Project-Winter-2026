import time
import board
import busio
from adafruit_pca9685 import PCA9685

# Initialize I2C and PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # Standard servo frequency

# Servo channels (0, 1, 2)
servo_channels = [0]

# Store angles
servo_angles = [90]
offsets = [0]

# Pulse range (adjust if needed)
SERVO_MIN = 150  # Min pulse
SERVO_MAX = 600  # Max pulse

def angle_to_pwm(angle):
    """Convert angle (0–180) to PWM value"""
    return int(SERVO_MIN + (angle / 180.0) * (SERVO_MAX - SERVO_MIN))

def set_servo(index, relative_angle):
    """Move servo using relative angle (based on offset)"""
    absolute_angle = offsets[index] + relative_angle

    if 0 <= absolute_angle <= 180:
        pwm_value = angle_to_pwm(absolute_angle)
        pca.channels[servo_channels[index]].duty_cycle = pwm_value << 4

        servo_angles[index] = absolute_angle

        print(f"Servo {index+1}: {relative_angle}° (relative), {absolute_angle}° (absolute)")
    else:
        print("⚠️ Angle out of range (0–180 after offset)")

def calibrate_zero():
    """Set current position as 0°"""
    global offsets
    offsets = servo_angles.copy()
    print("✅ Current positions set as new 0°")

def show_angles():
    print("\n=== Servo Angles ===")
    for i in range(3):
        relative = servo_angles[i] - offsets[i]
        print(f"Servo {i+1}: {relative}° (relative) | {servo_angles[i]}° (absolute)")
    print("====================\n")

# Main loop
try:
    while True:
        show_angles()

        cmd = input("Enter: servo(1-3), angle | 'z' = zero | 'q' = quit: ")

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
    pca.deinit()
    print("Cleaned up.")