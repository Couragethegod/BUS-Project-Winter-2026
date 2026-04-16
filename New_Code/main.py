# ball_balancer_pi.py
#
# Raspberry Pi + Pi Camera + 3 MG996R servos
# Ball balancer starter code
#
# What it does:
# 1. Captures frames from the Pi camera
# 2. Detects a colored ball
# 3. Computes X/Y error from the center of the platform
# 4. Uses PID control to tilt the platform
# 5. Mixes X/Y tilt into 3 servo commands spaced 120 degrees apart

import time
import cv2
import numpy as np
import pigpio
from picamera2 import Picamera2


# =========================
# USER SETTINGS
# =========================

# GPIO pins for the 3 servo signal wires
SERVO_PINS = [0, 1, 2]

# Servo calibration in microseconds
# Adjust these for your exact mechanism
SERVO_MIN_US = [55, 55, 55]
SERVO_MID_US = [75, 75, 75]
SERVO_MAX_US = [80, 80, 80]

# Flip individual servos if one moves opposite to what you expect
SERVO_DIR = [1, 1, 1]

# How strongly each servo responds to normalized command [-1..1]
SERVO_GAIN_US = [180, 180, 180]

# Camera size
FRAME_W = 640
FRAME_H = 480

# Ball target point = center of image
TARGET_X = FRAME_W // 2
TARGET_Y = FRAME_H // 2

# HSV threshold for the ball color
# Example below is for a yellow ball. Change these for your ball.
LOWER_HSV = np.array([20, 100, 100])
UPPER_HSV = np.array([30, 255, 255])

BALL_MIN_RADIUS = 8
DEADBAND_PX = 8

# If the platform moves the wrong way, flip these
TILT_SIGN_X = 1.0
TILT_SIGN_Y = 1.0

# PID gains
KP_X = 0.010
KI_X = 0.0000
KD_X = 0.0040

KP_Y = 0.010
KI_Y = 0.0000
KD_Y = 0.0040

# Clamp PID outputs to this range
CTRL_LIMIT = 1.0
INTEGRAL_LIMIT = 1000.0

# Position smoothing for noisy ball measurements
SMOOTH_ALPHA = 0.25

SHOW_WINDOWS = True


# =========================
# HELPER FUNCTIONS
# =========================

def clamp(value, low, high):
    return max(low, min(high, value))


class PID:
    def __init__(self, kp, ki, kd, out_limit=1.0, int_limit=1000.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_limit = out_limit
        self.int_limit = int_limit
        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error, dt):
        if dt <= 0:
            return 0.0

        self.integral += error * dt
        self.integral = clamp(self.integral, -self.int_limit, self.int_limit)

        if self.prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.prev_error) / dt

        self.prev_error = error

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )
        return clamp(output, -self.out_limit, self.out_limit)


def set_servo_command(pi, servo_index, normalized_cmd):
    """
    normalized_cmd in range [-1, +1]
    """
    normalized_cmd = clamp(normalized_cmd, -1.0, 1.0)

    pulse = (
        SERVO_MID_US[servo_index]
        + SERVO_DIR[servo_index] * SERVO_GAIN_US[servo_index] * normalized_cmd
    )

    pulse = clamp(pulse, SERVO_MIN_US[servo_index], SERVO_MAX_US[servo_index])
    pi.set_servo_pulsewidth(SERVO_PINS[servo_index], pulse)


def level_platform(pi):
    for i in range(3):
        pi.set_servo_pulsewidth(SERVO_PINS[i], SERVO_MID_US[i])


def release_servos(pi):
    for pin in SERVO_PINS:
        pi.set_servo_pulsewidth(pin, 0)


def detect_ball(frame):
    """
    Returns:
        (cx, cy, radius), mask
        or (None, mask) if no ball found
    """
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, mask

    c = max(contours, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    M = cv2.moments(c)

    if M["m00"] == 0 or radius < BALL_MIN_RADIUS:
        return None, mask

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    return (cx, cy, radius), mask


def mix_xy_to_3_servos(ux, uy):
    """
    Convert desired X/Y tilt into 3 servo commands.
    Assumes servos are mechanically spaced 120 degrees apart.

    Output range is roughly [-1, 1] before final clamp.
    """
    s1 = ux
    s2 = -0.5 * ux + 0.866 * uy
    s3 = -0.5 * ux - 0.866 * uy
    return [s1, s2, s3]


# =========================
# MAIN
# =========================

def main():
    # pigpio connection
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio is not running. Start it first with: sudo pigpiod")

    # Initialize servos to center
    level_platform(pi)
    time.sleep(1.0)

    # Camera setup
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (FRAME_W, FRAME_H), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)

    pid_x = PID(KP_X, KI_X, KD_X, out_limit=CTRL_LIMIT, int_limit=INTEGRAL_LIMIT)
    pid_y = PID(KP_Y, KI_Y, KD_Y, out_limit=CTRL_LIMIT, int_limit=INTEGRAL_LIMIT)

    filt_x = float(TARGET_X)
    filt_y = float(TARGET_Y)

    last_time = time.time()

    try:
        while True:
            frame = picam2.capture_array("main")
            ball, mask = detect_ball(frame)

            now = time.time()
            dt = now - last_time
            last_time = now

            ux = 0.0
            uy = 0.0

            if ball is not None:
                cx, cy, radius = ball

                # Smooth the measured position
                filt_x = SMOOTH_ALPHA * cx + (1.0 - SMOOTH_ALPHA) * filt_x
                filt_y = SMOOTH_ALPHA * cy + (1.0 - SMOOTH_ALPHA) * filt_y

                error_x = TARGET_X - filt_x
                error_y = TARGET_Y - filt_y

                # Deadband
                if abs(error_x) < DEADBAND_PX:
                    error_x = 0.0
                if abs(error_y) < DEADBAND_PX:
                    error_y = 0.0

                # PID
                ux = TILT_SIGN_X * pid_x.update(error_x, dt)
                uy = TILT_SIGN_Y * pid_y.update(error_y, dt)

                # Convert X/Y tilt to 3 servos
                servo_cmds = mix_xy_to_3_servos(ux, uy)

                # Apply commands
                for i in range(3):
                    set_servo_command(pi, i, servo_cmds[i])

                if SHOW_WINDOWS:
                    cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
                    cv2.circle(frame, (int(filt_x), int(filt_y)), 4, (0, 0, 255), -1)

            else:
                # No ball found: level platform and reset PID
                pid_x.reset()
                pid_y.reset()
                level_platform(pi)

            if SHOW_WINDOWS:
                # Draw target center
                cv2.line(frame, (TARGET_X - 15, TARGET_Y), (TARGET_X + 15, TARGET_Y), (255, 0, 0), 2)
                cv2.line(frame, (TARGET_X, TARGET_Y - 15), (TARGET_X, TARGET_Y + 15), (255, 0, 0), 2)

                # Status text
                cv2.putText(frame, f"ux={ux:.2f} uy={uy:.2f}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(frame, "Press q to quit", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                cv2.imshow("Ball Balancer", frame)
                cv2.imshow("Mask", mask)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

    finally:
        level_platform(pi)
        time.sleep(0.5)
        release_servos(pi)
        picam2.stop()
        cv2.destroyAllWindows()
        pi.stop()


if __name__ == "__main__":
    main()