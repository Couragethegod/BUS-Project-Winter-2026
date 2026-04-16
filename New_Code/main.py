from __future__ import annotations

import math
import time
from dataclasses import dataclass

import cv2
import numpy as np

from PID import PIDcontroller
from robot_kinematics import RobotKinematics

try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    from picamera2 import Picamera2
except ImportError as exc:  # pragma: no cover - depends on Raspberry Pi hardware libs
    HARDWARE_IMPORT_ERROR = exc
else:
    HARDWARE_IMPORT_ERROR = None


FRAME_SIZE = (640, 480)
CAMERA_FLIP = -1
SHOW_MASK_WINDOW = True

SAFE_SERVO_MIN_DEG = 45.0
SAFE_SERVO_MAX_DEG = 80.0
SAFE_SERVO_HOME_DEG = 62.5
PCA9685_FREQUENCY_HZ = 50
SERVO_MIN_PULSE_US = 500
SERVO_MAX_PULSE_US = 2500

PLATFORM_HEIGHT_CM = 8.26
MAX_PLATFORM_TILT_DEG = 8.0
POSITION_DEADBAND_PX = 4.0
BALL_LOST_TIMEOUT_S = 0.35

# If a servo or axis moves the wrong way on your build, flip the sign to -1.0.
CONTROL_SIGN_X = 1.0
CONTROL_SIGN_Y = 1.0

TEAL_LOWER_HSV = np.array([75, 70, 45], dtype=np.uint8)
TEAL_UPPER_HSV = np.array([105, 255, 255], dtype=np.uint8)

KINEMATICS = {
    "lp": 7.0,
    "l1": 10.0,
    "l2": 9.6,
    "lb": 7.4,
}


@dataclass(frozen=True)
class ServoCalibration:
    channel: int
    center_deg: float = SAFE_SERVO_HOME_DEG
    direction: float = 1.0
    trim_deg: float = 0.0


@dataclass
class BallDetection:
    center: tuple[int, int]
    radius: float
    area: float


SERVO_CALIBRATION = (
    ServoCalibration(channel=0, center_deg=62.0),
    ServoCalibration(channel=1, center_deg=62.5),
    ServoCalibration(channel=2, center_deg=62.5),
)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class UnderPlatformCamera:
    def __init__(self, resolution=FRAME_SIZE, flip=CAMERA_FLIP):
        self.flip = flip
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": resolution, "format": "RGB888"},
            controls={"FrameDurationLimits": (33333, 33333), "ExposureTime": 8000},
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1.0)

    def capture_frame(self):
        frame_rgb = self.picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        if self.flip is not None:
            frame_bgr = cv2.flip(frame_bgr, self.flip)
        return frame_bgr

    def close(self):
        self.picam2.stop()
        self.picam2.close()


class TealBallTracker:
    def __init__(self, lower_hsv=TEAL_LOWER_HSV, upper_hsv=TEAL_UPPER_HSV):
        self.lower_hsv = lower_hsv
        self.upper_hsv = upper_hsv
        self.kernel = np.ones((5, 5), dtype=np.uint8)

    def detect(self, frame_bgr):
        blurred = cv2.GaussianBlur(frame_bgr, (7, 7), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, mask

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < 180:
            return None, mask

        (x_pos, y_pos), radius = cv2.minEnclosingCircle(contour)
        if radius < 7:
            return None, mask

        moments = cv2.moments(contour)
        if moments["m00"]:
            center = (
                int(moments["m10"] / moments["m00"]),
                int(moments["m01"] / moments["m00"]),
            )
        else:
            center = (int(x_pos), int(y_pos))

        return BallDetection(center=center, radius=radius, area=area), mask


class PCA9685ServoController:
    def __init__(self, servo_calibration):
        self.servo_calibration = servo_calibration
        self.current_angles = [cal.center_deg for cal in self.servo_calibration]

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = PCA9685_FREQUENCY_HZ

    def write_angles(self, angles_deg):
        applied = []
        for calibration, angle_deg in zip(self.servo_calibration, angles_deg):
            safe_angle = clamp(angle_deg, SAFE_SERVO_MIN_DEG, SAFE_SERVO_MAX_DEG)
            pulse_us = SERVO_MIN_PULSE_US + (
                safe_angle / 180.0
            ) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)
            period_us = 1_000_000.0 / PCA9685_FREQUENCY_HZ
            duty_cycle = int((pulse_us / period_us) * 65535.0)
            self.pca.channels[calibration.channel].duty_cycle = clamp(duty_cycle, 0, 65535)
            applied.append(safe_angle)

        self.current_angles = applied
        return tuple(applied)

    def ramp_to(self, target_angles, duration=0.5, steps=25):
        start_angles = list(self.current_angles)
        for step in range(1, steps + 1):
            blend = step / steps
            interp_angles = [
                start + ((target - start) * blend)
                for start, target in zip(start_angles, target_angles)
            ]
            self.write_angles(interp_angles)
            time.sleep(duration / steps)

    def close(self):
        for calibration in self.servo_calibration:
            self.pca.channels[calibration.channel].duty_cycle = 0
        self.pca.deinit()


class SmartBallBalancer:
    def __init__(self):
        self.camera = UnderPlatformCamera()
        self.tracker = TealBallTracker()
        self.kinematics = RobotKinematics(**KINEMATICS)
        self.servos = PCA9685ServoController(SERVO_CALIBRATION)

        self.pid_x = PIDcontroller(
            kp=0.025,
            ki=0.0009,
            kd=0.012,
            windup_limit=250.0,
            output_limit=MAX_PLATFORM_TILT_DEG,
        )
        self.pid_y = PIDcontroller(
            kp=0.025,
            ki=0.0009,
            kd=0.012,
            windup_limit=250.0,
            output_limit=MAX_PLATFORM_TILT_DEG,
        )

        self.home_joint_angles = self.solve_joint_angles(0.0, 0.0)
        self.home_servo_angles = self.joint_angles_to_servo_angles(self.home_joint_angles)
        self.servos.ramp_to(self.home_servo_angles, duration=0.6, steps=30)
        self.last_detection_time = time.perf_counter()

    def solve_joint_angles(self, tilt_deg, direction_deg):
        self.kinematics.solve_inverse_kinematics_spherical(
            tilt_deg,
            direction_deg,
            PLATFORM_HEIGHT_CM,
        )
        return self.kinematics.joint_angles_deg()

    def joint_angles_to_servo_angles(self, joint_angles_deg):
        servo_angles = []
        for calibration, home_joint, joint_angle in zip(
            SERVO_CALIBRATION,
            self.home_joint_angles,
            joint_angles_deg,
        ):
            delta_deg = joint_angle - home_joint
            command_deg = (
                calibration.center_deg
                + (calibration.direction * delta_deg)
                + calibration.trim_deg
            )
            servo_angles.append(command_deg)
        return tuple(servo_angles)

    def command_platform(self, tilt_deg, direction_deg):
        safe_tilt = min(max(tilt_deg, 0.0), MAX_PLATFORM_TILT_DEG)
        joint_angles = self.solve_joint_angles(safe_tilt, direction_deg)
        servo_angles = self.joint_angles_to_servo_angles(joint_angles)
        applied_angles = self.servos.write_angles(servo_angles)
        return applied_angles, joint_angles

    def control_step(self, frame_bgr):
        target = (frame_bgr.shape[1] // 2, frame_bgr.shape[0] // 2)
        detection, mask = self.tracker.detect(frame_bgr)

        theta = 0.0
        phi = 0.0
        error_x = 0.0
        error_y = 0.0
        servo_angles = tuple(self.servos.current_angles)

        if detection is not None:
            self.last_detection_time = time.perf_counter()
            error_x = float(target[0] - detection.center[0])
            error_y = float(target[1] - detection.center[1])

            if abs(error_x) < POSITION_DEADBAND_PX:
                error_x = 0.0
            if abs(error_y) < POSITION_DEADBAND_PX:
                error_y = 0.0

            tilt_x = CONTROL_SIGN_X * self.pid_x.update(error_x)
            tilt_y = CONTROL_SIGN_Y * self.pid_y.update(error_y)

            theta = min(math.hypot(tilt_x, tilt_y), MAX_PLATFORM_TILT_DEG)
            if theta > 0.0:
                phi = math.degrees(math.atan2(tilt_y, tilt_x)) % 360.0

            servo_angles, _ = self.command_platform(theta, phi)
        elif time.perf_counter() - self.last_detection_time > BALL_LOST_TIMEOUT_S:
            self.pid_x.reset()
            self.pid_y.reset()
            servo_angles, _ = self.command_platform(0.0, 0.0)

        annotated = self.draw_overlay(
            frame_bgr,
            detection,
            target,
            error_x,
            error_y,
            theta,
            phi,
            servo_angles,
        )
        return annotated, mask

    def draw_overlay(
        self,
        frame_bgr,
        detection,
        target,
        error_x,
        error_y,
        theta,
        phi,
        servo_angles,
    ):
        display = frame_bgr.copy()

        cv2.drawMarker(
            display,
            target,
            color=(0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=20,
            thickness=2,
        )

        if detection is not None:
            cv2.circle(display, detection.center, int(detection.radius), (0, 255, 0), 2)
            cv2.line(display, target, detection.center, (255, 255, 255), 2)
            status_text = "Tracking"
        else:
            status_text = "Searching"

        cv2.putText(
            display,
            f"Status: {status_text}",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            display,
            f"Error px: ({error_x:.1f}, {error_y:.1f})",
            (12, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            display,
            f"Tilt: {theta:.2f} deg @ {phi:.1f} deg",
            (12, 88),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            display,
            f"Servos: {servo_angles[0]:.1f}, {servo_angles[1]:.1f}, {servo_angles[2]:.1f}",
            (12, 118),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        return display

    def shutdown(self):
        try:
            self.servos.ramp_to(self.home_servo_angles, duration=0.4, steps=20)
        finally:
            self.camera.close()
            self.servos.close()
            cv2.destroyAllWindows()


def main():
    if HARDWARE_IMPORT_ERROR is not None:
        raise RuntimeError(
            "This script must run on the Raspberry Pi with picamera2, board, busio, and "
            f"adafruit_pca9685 installed. Missing dependency: {HARDWARE_IMPORT_ERROR}"
        )

    robot = SmartBallBalancer()

    try:
        while True:
            frame = robot.camera.capture_frame()
            annotated, mask = robot.control_step(frame)

            cv2.imshow("Smart Ball Balancer", annotated)
            if SHOW_MASK_WINDOW:
                cv2.imshow("Teal Ball Mask", mask)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        robot.shutdown()


if __name__ == "__main__":
    main()
