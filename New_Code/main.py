import math
import time

import cv2
import numpy as np

from PID import PIDcontroller
from robot_kinematics import RobotKinematics

try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    from picamera2 import Picamera2
except ImportError as exc:  # pragma: no cover - Raspberry Pi only
    HARDWARE_IMPORT_ERROR = exc
else:
    HARDWARE_IMPORT_ERROR = None


FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_FLIP = -1
SHOW_MASK_WINDOW = True

SERVO_CHANNELS = [0, 2, 1]
SERVO_CENTER_ANGLES = [62.5, 62.5, 62.5]
SERVO_DIRECTIONS = [1.0, 1.0, 1.0]
SERVO_TRIMS = [0.0, 0.0, 0.0]
SERVO_MOTION_GAIN = 3.0

SAFE_SERVO_MIN = 45.0
SAFE_SERVO_MAX = 80.0
SERVO_FREQUENCY = 50
SERVO_MIN_PULSE_US = 500
SERVO_MAX_PULSE_US = 2500

PLATFORM_HEIGHT = 8.26
MAX_TILT = 10.0
POSITION_DEADBAND = 3.0
CENTER_TARGET_RADIUS = 18.0
MIN_CENTERING_TILT = 2.5
BALL_LOST_TIMEOUT = 0.35
TILT_RESPONSE_GAIN = 1.8

CONTROL_SIGN_X = 1.0
CONTROL_SIGN_Y = 1.0

TEAL_LOWER = np.array([90, 100, 100], dtype=np.uint8)
TEAL_UPPER = np.array([110, 255, 255], dtype=np.uint8)
MIN_BALL_AREA = 40
MIN_BALL_RADIUS = 4
MASK_KERNEL = np.ones((5, 5), dtype=np.uint8)

PID_KP = 0.08
PID_KI = 0.0015
PID_KD = 0.015
"""PID_KP = 0.08
PID_KI = 0.0015
PID_KD = 0.015"""
PID_INTEGRAL_LIMIT = 300.0


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def setup_camera():
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"},
        controls={"FrameDurationLimits": (33333, 33333), "ExposureTime": 12000},
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)
    return picam2


def get_frame(picam2):
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if CAMERA_FLIP is not None:
        frame = cv2.flip(frame, CAMERA_FLIP)
    return frame


def setup_servos():
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = SERVO_FREQUENCY
    return pca


def set_servo_angle(pca, channel, angle):
    safe_angle = clamp(angle, SAFE_SERVO_MIN, SAFE_SERVO_MAX)
    pulse_us = SERVO_MIN_PULSE_US + (safe_angle / 180.0) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)
    period_us = 1_000_000.0 / SERVO_FREQUENCY
    duty_cycle = int((pulse_us / period_us) * 65535.0)
    pca.channels[channel].duty_cycle = clamp(duty_cycle, 0, 65535)
    return safe_angle


def set_all_servos(pca, angles):
    applied = []
    for channel, angle in zip(SERVO_CHANNELS, angles):
        applied.append(set_servo_angle(pca, channel, angle))
    return tuple(applied)


def find_teal_ball(frame):
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, TEAL_LOWER, TEAL_UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MASK_KERNEL, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MASK_KERNEL, iterations=3)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask

    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < MIN_BALL_AREA:
        return None, mask

    (x_pos, y_pos), radius = cv2.minEnclosingCircle(contour)
    if radius < MIN_BALL_RADIUS:
        return None, mask

    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        ball_x = int(x_pos)
        ball_y = int(y_pos)
    else:
        ball_x = int(moments["m10"] / moments["m00"])
        ball_y = int(moments["m01"] / moments["m00"])

    return (ball_x, ball_y, radius, area), mask


def get_home_joint_angles(model):
    model.solve_inverse_kinematics_spherical(0.0, 0.0, PLATFORM_HEIGHT)
    return model.joint_angles_deg()


def platform_to_servo_angles(model, home_joint_angles, tilt_deg, direction_deg):
    model.solve_inverse_kinematics_spherical(tilt_deg, direction_deg, PLATFORM_HEIGHT)
    joint_angles = model.joint_angles_deg()

    servo_angles = []
    for index in range(3):
        joint_change = joint_angles[index] - home_joint_angles[index]
        servo_angle = (
            SERVO_CENTER_ANGLES[index]
            + SERVO_DIRECTIONS[index] * joint_change * SERVO_MOTION_GAIN
            + SERVO_TRIMS[index]
        )
        servo_angles.append(servo_angle)

    return tuple(servo_angles)


def draw_overlay(frame, target, ball, error_x, error_y, distance_error, theta, phi, servo_angles):
    display = frame.copy()

    cv2.drawMarker(
        display,
        target,
        color=(0, 0, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=20,
        thickness=2,
    )

    if ball is not None:
        ball_x, ball_y, radius, area = ball
        cv2.circle(display, (ball_x, ball_y), int(radius), (0, 255, 0), 2)
        cv2.line(display, target, (ball_x, ball_y), (255, 255, 255), 2)
        status = "Tracking"
    else:
        area = 0.0
        radius = 0.0
        status = "Searching"

    cv2.putText(display, f"Status: {status}", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(display, f"Error: ({error_x:.1f}, {error_y:.1f})", (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(display, f"Tilt: {theta:.2f} deg  Phi: {phi:.1f}", (12, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(
        display,
        f"Servos: {servo_angles[0]:.1f}, {servo_angles[1]:.1f}, {servo_angles[2]:.1f}",
        (12, 118),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )
    cv2.putText(display, f"Ball area: {area:.0f}  radius: {radius:.1f}", (12, 148), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(display, f"Distance from center: {distance_error:.1f}px", (12, 178), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return display


def main():
    if HARDWARE_IMPORT_ERROR is not None:
        raise RuntimeError(
            "Run this on the Raspberry Pi with picamera2, board, busio, and adafruit_pca9685 installed. "
            f"Missing dependency: {HARDWARE_IMPORT_ERROR}"
        )

    picam2 = setup_camera()
    pca = setup_servos()
    model = RobotKinematics(lp=7.0, l1=10.0, l2=9.6, lb=7.4)

    pid_x = PIDcontroller(kp=PID_KP, ki=PID_KI, kd=PID_KD, integral_limit=PID_INTEGRAL_LIMIT, output_limit=MAX_TILT)
    pid_y = PIDcontroller(kp=PID_KP, ki=PID_KI, kd=PID_KD, integral_limit=PID_INTEGRAL_LIMIT, output_limit=MAX_TILT)

    home_joint_angles = get_home_joint_angles(model)
    home_servo_angles = tuple(
        SERVO_CENTER_ANGLES[index] + SERVO_TRIMS[index]
        for index in range(3)
    )
    current_servo_angles = set_all_servos(pca, home_servo_angles)
    last_ball_time = time.perf_counter()

    try:
        while True:
            frame = get_frame(picam2)
            target = (frame.shape[1] // 2, frame.shape[0] // 2)
            ball, mask = find_teal_ball(frame)

            error_x = 0.0
            error_y = 0.0
            distance_error = 0.0
            theta = 0.0
            phi = 0.0

            if ball is not None:
                last_ball_time = time.perf_counter()
                ball_x, ball_y, _, _ = ball

                error_x = float(target[0] - ball_x)
                error_y = float(target[1] - ball_y)

                if abs(error_x) < POSITION_DEADBAND:
                    error_x = 0.0
                if abs(error_y) < POSITION_DEADBAND:
                    error_y = 0.0

                distance_error = math.hypot(error_x, error_y)
                tilt_x = CONTROL_SIGN_X * TILT_RESPONSE_GAIN * pid_x.update(error_x)
                tilt_y = CONTROL_SIGN_Y * TILT_RESPONSE_GAIN * pid_y.update(error_y)

                theta = min(math.hypot(tilt_x, tilt_y), MAX_TILT)
                if theta > 0.0:
                    phi = math.degrees(math.atan2(tilt_y, tilt_x)) % 360.0
                    if distance_error > CENTER_TARGET_RADIUS:
                        theta = max(theta, MIN_CENTERING_TILT)

                try:
                    target_servo_angles = platform_to_servo_angles(model, home_joint_angles, theta, phi)
                    current_servo_angles = set_all_servos(pca, target_servo_angles)
                except ValueError:
                    pid_x.reset()
                    pid_y.reset()
                    current_servo_angles = set_all_servos(pca, home_servo_angles)
                    theta = 0.0
                    phi = 0.0

            elif time.perf_counter() - last_ball_time > BALL_LOST_TIMEOUT:
                pid_x.reset()
                pid_y.reset()
                current_servo_angles = set_all_servos(pca, home_servo_angles)

            display = draw_overlay(frame, target, ball, error_x, error_y, distance_error, theta, phi, current_servo_angles)
            cv2.imshow("Smart Ball Balancer", display)
            if SHOW_MASK_WINDOW:
                cv2.imshow("Teal Ball Mask", mask)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break

    finally:
        set_all_servos(pca, home_servo_angles)
        time.sleep(0.3)
        picam2.stop()
        picam2.close()
        pca.deinit()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
