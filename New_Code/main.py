# ball_balancer_pca9685.py
#
# Raspberry Pi + Pi Camera + PCA9685 + 3 MG996R servos
# Ball balancer starter code using PCA9685

import time
import cv2
import numpy as np
from adafruit_pca9685 import PCA9685
from picamera2 import Picamera2
import board
import busio

# =========================
# USER SETTINGS
# =========================

# I2C address of the PCA9685 board
PCA9685_ADDRESS = 0x40

# Servo calibration in degrees (adjust as needed)
SERVO_MIN = 40  # Minimum PWM value (usually 0)
SERVO_MAX = 80  # Maximum PWM value (usually 180)

# Camera size
FRAME_W = 640
FRAME_H = 480

# Ball target point = center of image
TARGET_X = FRAME_W // 2
TARGET_Y = FRAME_H // 2

# HSV threshold for the ball color
LOWER_HSV = np.array([20, 100, 100])
UPPER_HSV = np.array([30, 255, 255])

BALL_MIN_RADIUS = 8
DEADBAND_PX = 8

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


def level_platform(pwm):
    """
    Center the platform by setting all servos to the middle position (90 degrees).
    """
    for i in range(3):
        pwm.set_pwm(i, 0, int(90 / 180 * 4095))  # Convert 90 degrees to PWM value


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
    # Setup I2C communication
    i2c = busio.I2C(board.SCL, board.SDA)
    pwm = PCA9685(i2c)
    pwm.frequency = 60  # Set the PWM frequency for servos (typical value for servos)

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
                ux = pid_x.update(error_x, dt)
                uy = pid_y.update(error_y, dt)

                # Convert X/Y tilt to 3 servos
                servo_cmds = mix_xy_to_3_servos(ux, uy)

                # Apply commands to the PCA9685 (map [-1, 1] to PWM range)
                for i in range(3):
                    pwm.set_pwm(i, 0, int((servo_cmds[i] * 90 + 90) / 180 * 4095))  # Map to PWM values

            else:
                # No ball found: level platform and reset PID
                pid_x.reset()
                pid_y.reset()
                level_platform(pwm)

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
        level_platform(pwm)
        time.sleep(0.5)
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()