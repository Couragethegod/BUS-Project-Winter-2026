# Hardware + timing libraries
import board
import time
import busio

# Servo driver libraries (Adafruit PCA9685)
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit

# Your robot kinematics model
from robotKinematics import RobotKinematics

import math
import random


def clamp(value, lower=0, upper=90):
    """
    Restrict a value between lower and upper bounds.
    
    This prevents sending unsafe angles to the servo.
    
    🔧 Adjust limits if your servo allows more range
    """
    return max(lower, min(value, upper))


class RobotController:
    def __init__(self, model, lp=7.125, l1=6.20, l2=4.50, lb=4.00):
        """
        Robot Controller Initialization
        
        model: RobotKinematics object
        lp, l1, l2, lb: robot geometry (not used here if model is passed)
        """

        # Store kinematics model (handles math)
        self.robot = model
        
        # Initialize PCA9685 servo controller (16 channels)
        self.Controller = ServoKit(channels=16)

        # Assign servos to specific channels
        # 🔧 CHANGE THESE if wiring is different
        self.s1 = self.Controller.servo[0]
        self.s2 = self.Controller.servo[1]
        self.s3 = self.Controller.servo[2]

        # Configure servo properties
        for s in (self.s1, self.s2, self.s3):
            s.actuation_range = 270           # Max rotation range (degrees)
            s.set_pulse_width_range(500, 2500)  # PWM range (microseconds)

        # Move robot to safe starting position
        self.initialize()


    def initialize(self):
        """
        Initial startup sequence
        
        Moves robot to a known safe configuration
        """

        print("Initializing ...")

        # Move to mid position
        self.set_motor_angles(67, 67, 67)

        # Slowly move to minimum angles
        self.interpolate_time([55, 55, 55], duration=0.25)

        time.sleep(1)

        # Move to neutral upright posture
        self.Goto_time_spherical(0, 0, 8.26, t=0.25)

        time.sleep(1)

        print("Initialized!")
    

    def set_motor_angles(self, theta1, theta2, theta3):
        """
        Directly set servo angles
        
        theta1, theta2, theta3: angles in degrees
        """

        # Apply safety clamp + calibration offsets
        self.s1.angle = clamp(theta1) - 4   # Offset correction for servo 1
        self.s2.angle = clamp(theta2)
        self.s3.angle = clamp(theta3)


    def interpolate_time(self, target_angles, steps=100, duration=0.3, individual_durations=None):
        """
        Smoothly move servos to target angles over time
        
        target_angles: [θ1, θ2, θ3]
        duration: total movement time (seconds)
        
        individual_durations:
            Optional per-servo timing
        """

        # Current servo angles
        current_angles = [self.s1.angle, self.s2.angle, self.s3.angle]

        # If not provided, all servos use same duration
        if individual_durations is None:
            individual_durations = [duration] * 3

        # Use longest duration
        max_duration = max(individual_durations)

        # Number of interpolation steps (~100 Hz update)
        steps = max(1, int(max_duration / 0.01))

        # Interpolation loop
        for i in range(steps + 1):
            t = i * max_duration / steps

            # Compute intermediate angles
            angles = [
                c + (t_angle - c) * min(t / d, 1) if d > 0 else t_angle 
                for c, t_angle, d in zip(current_angles, target_angles, individual_durations)
            ]

            # Apply angles
            self.set_motor_angles(*angles)

            # Wait for next step
            time.sleep(max_duration / steps)


    def interpolate_speed(self, target_angles, speed=30, individual_speeds=None):
        """
        Move servos at a given speed (deg/sec)
        
        speed: degrees per second
        """

        current_angles = [self.s1.angle, self.s2.angle, self.s3.angle]

        if individual_speeds is None:
            individual_speeds = [speed] * 3

        # Compute time required for each servo
        durations = [
            abs(t - c) / s if s > 0 else 0 
            for c, t, s in zip(current_angles, target_angles, individual_speeds)
        ]

        max_duration = max(durations)
        steps = max(1, int(max_duration / 0.01))

        for i in range(steps + 1):
            t = i * max_duration / steps

            angles = [
                c + (t_angle - c) * min(t / d, 1) if d > 0 else t_angle 
                for c, t_angle, d in zip(current_angles, target_angles, durations)
            ]

            self.set_motor_angles(*angles)
            time.sleep(max_duration / steps)


    def Goto_time_spherical(self, theta, phi, h, t=0.5):
        """
        Move robot using spherical coordinates
        
        theta: direction (degrees)
        phi: tilt angle (degrees)
        h: height
        t: time to move
        """

        # Solve inverse kinematics
        self.robot.solve_inverse_kinematics_spherical(theta, phi, h)

        # Convert radians → servo angles
        target_angles = [
            math.degrees(math.pi*0.5 - self.robot.theta1),
            math.degrees(math.pi*0.5 - self.robot.theta2),
            math.degrees(math.pi*0.5 - self.robot.theta3)
        ]

        # Move smoothly
        self.interpolate_time(target_angles, duration=t)


    def Goto_time_vector(self, a, b, c, h, t=0.5):
        """
        Move robot using vector direction
        
        (a, b, c) = normal vector
        h = height
        """

        self.robot.solve_inverse_kinematics_vector(a, b, c, h)

        target_angles = [
            math.degrees(math.pi*0.5 - self.robot.theta1),
            math.degrees(math.pi*0.5 - self.robot.theta2),
            math.degrees(math.pi*0.5 - self.robot.theta3)
        ]

        self.interpolate_time(target_angles, duration=t)


    def Goto_N_time_vector(self, a, b, c, h):
        """
        Instant move (no interpolation)
        """
        self.robot.solve_inverse_kinematics_vector(a, b, c, h)

        target_angles = [
            math.degrees(math.pi*0.5 - self.robot.theta1),
            math.degrees(math.pi*0.5 - self.robot.theta2),
            math.degrees(math.pi*0.5 - self.robot.theta3)
        ]

        self.set_motor_angles(*target_angles)
    

    def Goto_N_time_spherical(self, theta, phi, h):
        """
        Instant move using spherical coordinates
        """

        self.robot.solve_inverse_kinematics_spherical(theta, phi, h)

        target_angles = [
            math.degrees(math.pi*0.5 - self.robot.theta1),
            math.degrees(math.pi*0.5 - self.robot.theta2),
            math.degrees(math.pi*0.5 - self.robot.theta3)
        ]

        self.set_motor_angles(*target_angles)


    def Dance1(self):
        """
        Demo motion: circular tilting pattern
        """

        # Move to starting tilt
        self.Goto_time_vector(0.2588, 0, 0.9659, 8)

        # Perform circular motion
        for _ in range(3):
            for i in range(100):
                t = (2 * math.pi / 100) * i

                x = math.cos(math.pi * 5 / 12) * math.cos(t)
                y = math.cos(math.pi * 5 / 12) * math.sin(t)
                z = math.sin(math.pi * 5 / 12)

                self.Goto_N_time_vector(x, y, z, 8)

                time.sleep(1/100)

        # Return to upright
        self.Goto_time_vector(0, 0, 1, 8)


# ------------------- MAIN -------------------

if __name__ == "__main__":

    # Create kinematics model
    model = RobotKinematics()

    # Create controller
    rc = RobotController(model)

    time.sleep(0.5)