import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from picamera2 import Picamera2
from adafruit_pca9685 import PCA9685
import board
import busio


from PID import PIDcontroller
from robot_kinematics import RobotKinematics

W, H = 640, 480
SERVO_CH = [0, 2, 1]
SERVO_HOME = [62.5, 62.5, 62.5]
SERVO_MIN, SERVO_MAX = 45.0, 80.0
PULSE_MIN, PULSE_MAX = 500, 2500
HEIGHT, MAX_TILT, ARM_GAIN = 8.26, 6.0, 1.8
LOWER = np.array([70, 45, 35], dtype=np.uint8)
UPPER = np.array([110, 255, 255], dtype=np.uint8)
SIGN_X, SIGN_Y = 1.0, 1.0


def clamp(x, a, b):
    return max(a, min(b, x))


def set_servo(pca, ch, angle):
    angle = clamp(angle, SERVO_MIN, SERVO_MAX)
    pulse = PULSE_MIN + (angle / 180.0) * (PULSE_MAX - PULSE_MIN)
    pca.channels[ch].duty_cycle = int((pulse / 20000.0) * 65535.0)
    return angle


def set_all(pca, angles):
    return tuple(set_servo(pca, ch, ang) for ch, ang in zip(SERVO_CH, angles))


def find_ball(frame):
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (7, 7), 0), cv2.COLOR_BGR2HSV)
    mask = cv2.morphologyEx(cv2.inRange(hsv, LOWER, UPPER), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < 50:
        return None, mask
    (x, y), r = cv2.minEnclosingCircle(c)
    m = cv2.moments(c)
    if r < 4:
        return None, mask
    x = int(m["m10"] / m["m00"]) if m["m00"] else int(x)
    y = int(m["m01"] / m["m00"]) if m["m00"] else int(y)
    return (x, y, r), mask


def main():
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"size": (W, H), "format": "RGB888"}))
    picam2.start()
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50

    model = RobotKinematics()
    model.solve_inverse_kinematics_spherical(0, 0, HEIGHT)
    home_joints = model.joint_angles_deg()
    angles = set_all(pca, SERVO_HOME)
    pid_x = PIDcontroller(0.04, 0.0008, 0.008, integral_limit=200, output_limit=MAX_TILT)
    pid_y = PIDcontroller(0.04, 0.0008, 0.008, integral_limit=200, output_limit=MAX_TILT)
    fx = fy = None

    try:
        while True:
            frame = cv2.flip(cv2.cvtColor(picam2.capture_array(), cv2.COLOR_RGB2BGR), -1)
            ball, mask = find_ball(frame)
            cx, cy = W // 2, H // 2
            theta = phi = ex = ey = 0.0

            if ball:
                x, y, r = ball
                fx = x if fx is None else 0.25 * x + 0.75 * fx
                fy = y if fy is None else 0.25 * y + 0.75 * fy
                ex, ey = cx - fx, cy - fy
                tilt_x = SIGN_X * pid_x.update(ex)
                tilt_y = SIGN_Y * pid_y.update(ey)
                theta = min(math.hypot(tilt_x, tilt_y), MAX_TILT)
                phi = math.degrees(math.atan2(tilt_y, tilt_x)) % 360 if theta else 0
                model.solve_inverse_kinematics_spherical(theta, phi, HEIGHT)
                joints = model.joint_angles_deg()
                target = [SERVO_HOME[i] + (joints[i] - home_joints[i]) * ARM_GAIN for i in range(3)]
                angles = set_all(pca, [0.35 * t + 0.65 * a for t, a in zip(target, angles)])
                cv2.circle(frame, (x, y), int(r), (0, 255, 0), 2)
                cv2.line(frame, (cx, cy), (int(fx), int(fy)), (255, 255, 255), 2)
            else:
                fx = fy = None
                pid_x.reset()
                pid_y.reset()
                angles = set_all(pca, [0.2 * h + 0.8 * a for h, a in zip(SERVO_HOME, angles)])

            cv2.drawMarker(frame, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(frame, f"err=({ex:.1f},{ey:.1f}) tilt={theta:.2f}", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"servos={angles[0]:.1f},{angles[1]:.1f},{angles[2]:.1f}", (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow("Ball Balancer", frame)
            cv2.imshow("Mask", mask)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break
    finally:
        set_all(pca, SERVO_HOME)
        time.sleep(0.3)
        picam2.stop()
        picam2.close()
        pca.deinit()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
