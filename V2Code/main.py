"""
Smart Ball Balancer Robot
Hardware: Raspberry Pi 4B, Pi Camera 2, PCA9685, 3x MG996R servos
Camera is mounted underneath a clear acrylic platform.

Servo safe range: 45–80 degrees (mid/flat = 62 degrees)
Servo layout: delta/tripod, evenly spaced at 120° apart
"""

import time
import math
import cv2
import numpy as np
from picamera2 import Picamera2
import smbus2

# ─────────────────────────────────────────────
# PCA9685 I2C driver (no external libraries)
# ─────────────────────────────────────────────

PCA9685_ADDRESS  = 0x40
MODE1_REG        = 0x00
PRESCALE_REG     = 0xFE
LED0_ON_L        = 0x06  # Base register; each channel adds 4

class PCA9685:
    def __init__(self, address=PCA9685_ADDRESS, bus=1):
        self.bus     = smbus2.SMBus(bus)
        self.address = address
        # Wake the chip (clear SLEEP bit)
        self.bus.write_byte_data(self.address, MODE1_REG, 0x00)
        time.sleep(0.005)
        self.set_pwm_freq(50)  # 50 Hz for servos

    def set_pwm_freq(self, freq_hz):
        """Set PWM frequency (50 Hz for standard servos)."""
        prescale = round(25_000_000 / (4096 * freq_hz)) - 1
        old_mode = self.bus.read_byte_data(self.address, MODE1_REG)
        # Enter sleep mode to set prescale
        self.bus.write_byte_data(self.address, MODE1_REG, (old_mode & 0x7F) | 0x10)
        self.bus.write_byte_data(self.address, PRESCALE_REG, prescale)
        # Restore mode and restart
        self.bus.write_byte_data(self.address, MODE1_REG, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, MODE1_REG, old_mode | 0x80)

    def set_pwm(self, channel, on, off):
        """Write raw on/off tick values for a channel (0–4095)."""
        base = LED0_ON_L + 4 * channel
        self.bus.write_byte_data(self.address, base + 0, on  & 0xFF)
        self.bus.write_byte_data(self.address, base + 1, on  >> 8)
        self.bus.write_byte_data(self.address, base + 2, off & 0xFF)
        self.bus.write_byte_data(self.address, base + 3, off >> 8)

    def set_servo_angle(self, channel, angle_deg):
        """
        Convert degrees to PCA9685 ticks at 50 Hz.
        Pulse width: 0° → ~0.5 ms, 180° → ~2.5 ms
        """
        # Clamp to safe mechanical range
        angle_deg = max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, angle_deg))

        pulse_ms  = 0.5 + (angle_deg / 180.0) * 2.0   # 0.5–2.5 ms
        tick      = int((pulse_ms / 20.0) * 4096)       # 20 ms period at 50 Hz
        self.set_pwm(channel, 0, tick)


# ─────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────

# Servo safe angle limits
SERVO_MIN_DEG = 45
SERVO_MAX_DEG = 80
SERVO_MID_DEG = (SERVO_MIN_DEG + SERVO_MAX_DEG) / 2   # 62.5° → flat platform

# PCA9685 channels for each servo
SERVO_CHANNELS = [0, 1, 2]

# Delta (tripod) servo arm angles relative to the platform axes.
# Servo 0 points "away" (top of frame), then 120° spacing.
# Adjust these if your physical layout differs.
SERVO_ARM_ANGLES_DEG = [270, 30, 150]

# Maximum tilt range in degrees (total swing from SERVO_MID)
MAX_TILT = (SERVO_MAX_DEG - SERVO_MID_DEG)   # ~17.5°

# Camera resolution
FRAME_W = 640
FRAME_H = 480

# Setpoint: centre of the platform in pixels
SETPOINT_X = FRAME_W // 2
SETPOINT_Y = FRAME_H // 2

# PID gains — tune these for your physical setup
KP = 0.05
KI = 0.001
KD = 0.02

# Yellow ball HSV range (tune with a colour picker if needed)
HSV_LOWER = np.array([20,  80,  80])
HSV_UPPER = np.array([35, 255, 255])

# Minimum blob area (px²) to count as the ball
MIN_BALL_AREA = 400


# ─────────────────────────────────────────────
# PID controller (simple, stateful)
# ─────────────────────────────────────────────

class PID:
    def __init__(self, kp, ki, kd):
        self.kp        = kp
        self.ki        = ki
        self.kd        = kd
        self.integral  = 0.0
        self.prev_err  = 0.0
        self.prev_time = time.time()

    def reset(self):
        self.integral  = 0.0
        self.prev_err  = 0.0
        self.prev_time = time.time()

    def compute(self, error):
        now = time.time()
        dt  = max(now - self.prev_time, 1e-4)

        self.integral += error * dt
        # Anti-windup: clamp integral contribution
        self.integral  = max(-50, min(50, self.integral))

        derivative     = (error - self.prev_err) / dt
        output         = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_err  = error
        self.prev_time = now
        return output


# ─────────────────────────────────────────────
# Ball detection
# ─────────────────────────────────────────────

def detect_ball(frame):
    """
    Detect the yellow ball using HSV colour masking.
    Returns (cx, cy) in pixels, or None if not found.
    """
    hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask   = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

    # Clean up noise
    mask   = cv2.erode (mask, None, iterations=2)
    mask   = cv2.dilate(mask, None, iterations=2)

    # Find contours and pick the largest one
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < MIN_BALL_AREA:
        return None

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return cx, cy


# ─────────────────────────────────────────────
# Platform / servo geometry
# ─────────────────────────────────────────────

def compute_servo_angles(tilt_x, tilt_y):
    """
    Convert a desired X/Y platform tilt (degrees) into
    individual servo angles using delta geometry.

    tilt_x: positive → platform tilts right  (ball moves right)
    tilt_y: positive → platform tilts forward (ball moves down)
    """
    angles = []
    for arm_deg in SERVO_ARM_ANGLES_DEG:
        arm_rad = math.radians(arm_deg)
        # Project the tilt vector onto this servo's axis
        contribution = tilt_x * math.cos(arm_rad) + tilt_y * math.sin(arm_rad)
        servo_angle  = SERVO_MID_DEG + contribution
        # Clamp to safe range
        servo_angle  = max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, servo_angle))
        angles.append(servo_angle)
    return angles


# ─────────────────────────────────────────────
# Main loop
# ─────────────────────────────────────────────

def main():
    print("Initialising PCA9685...")
    pca = PCA9685()

    # Move all servos to flat/neutral position at startup
    print(f"Moving to neutral position ({SERVO_MID_DEG:.1f}°)...")
    for ch in SERVO_CHANNELS:
        pca.set_servo_angle(ch, SERVO_MID_DEG)
    time.sleep(1.0)

    print("Starting camera...")
    cam = Picamera2()
    config = cam.create_preview_configuration(
        main={"size": (FRAME_W, FRAME_H), "format": "BGR888"}
    )
    cam.configure(config)
    cam.start()
    time.sleep(0.5)

    pid_x = PID(KP, KI, KD)
    pid_y = PID(KP, KI, KD)

    print("Running — press Ctrl+C to stop.")
    try:
        while True:
            frame = cam.capture_array()

            ball_pos = detect_ball(frame)

            if ball_pos is not None:
                bx, by = ball_pos

                # Error = displacement from centre (pixels)
                err_x =  (bx - SETPOINT_X)   # positive → ball is right of centre
                err_y =  (by - SETPOINT_Y)   # positive → ball is below centre

                # PID output → desired tilt (degrees)
                tilt_x = pid_x.compute(err_x)
                tilt_y = pid_y.compute(err_y)

                # Clamp tilt to physical range
                tilt_x = max(-MAX_TILT, min(MAX_TILT, tilt_x))
                tilt_y = max(-MAX_TILT, min(MAX_TILT, tilt_y))

                # Compute and apply servo angles
                servo_angles = compute_servo_angles(tilt_x, tilt_y)
                for ch, angle in zip(SERVO_CHANNELS, servo_angles):
                    pca.set_servo_angle(ch, angle)

                # Optional debug overlay
                cv2.circle(frame, (bx, by), 10, (0, 255, 0), 2)
                cv2.line(frame, (SETPOINT_X - 10, SETPOINT_Y),
                                (SETPOINT_X + 10, SETPOINT_Y), (0, 0, 255), 1)
                cv2.line(frame, (SETPOINT_X, SETPOINT_Y - 10),
                                (SETPOINT_X, SETPOINT_Y + 10), (0, 0, 255), 1)

            else:
                # Ball not visible — return to flat and reset integrators
                pid_x.reset()
                pid_y.reset()
                for ch in SERVO_CHANNELS:
                    pca.set_servo_angle(ch, SERVO_MID_DEG)

            # Uncomment to show live preview (requires a display)
            # cv2.imshow("Ball Balancer", frame)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

    except KeyboardInterrupt:
        print("\nStopping — returning to neutral.")

    finally:
        for ch in SERVO_CHANNELS:
            pca.set_servo_angle(ch, SERVO_MID_DEG)
        time.sleep(0.5)
        cam.stop()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()