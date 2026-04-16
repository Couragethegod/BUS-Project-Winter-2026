import board
import time
import busio
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
from robotKinematics import RobotKinematics
import math
import random

# UPDATED: Clamp range changed to 45 - 80
def clamp(value, lower=45, upper=80):
    return max(lower, min(value, upper))

class RobotController:
    def __init__(self, model, lp=7, l1=10, l2=9.6, lb=7.4):
        self.robot = model
        
        # Initialize the ServoKit and assign servos
        self.Controller = ServoKit(channels=16)
        self.s1 = self.Controller.servo[0]
<<<<<<< HEAD
        self.s2 = self.Controller.servo[0]
        self.s3 = self.Controller.servo[0]
=======
        self.s2 = self.Controller.servo[1]
        self.s3 = self.Controller.servo[2]
>>>>>>> 58a418f3208df2bc63e09467804a81e15c452191

        # Configure servos
        for s in (self.s1, self.s2, self.s3):
            s.actuation_range = 270
            s.set_pulse_width_range(500, 2500)

        self.initialize()

    def initialize(self):
        print("Initializing with new range [45, 80]...")
        # Start at a neutral mid-point (e.g., 60)
        self.set_motor_angles(60, 60, 60)
        # Move to the new minimum "bottom" position (45) instead of the old 19
        self.interpolate_time([45, 45, 45], duration=0.25)
        time.sleep(1)
        # Move to the working height (8.26)
        self.Goto_time_spherical(0, 0, 8.26, t=0.25)
        time.sleep(1)
        print("Initialized!")
    
    def set_motor_angles(self, theta1, theta2, theta3):
        # The clamp function here now enforces the 45-80 limit
        self.s1.angle = clamp(theta1) - 4 
        self.s2.angle = clamp(theta2) 
        self.s3.angle = clamp(theta3) 

    # ... (rest of the interpolation methods remain the same)