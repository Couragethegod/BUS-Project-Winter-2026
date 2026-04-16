from picamera2 import Picamera2
import numpy as np
import time
import math
import cv2
from controller import RobotController
from robotKinematics import RobotKinematics
from PID import PIDcontroller

# Initialize Picamera2
picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration())

# Start the camera
picam2.start()

# Initialize robot model and controller
model = RobotKinematics()
robot = RobotController(model)

# PID controller settings
pid = PIDcontroller(kp=0.0063, ki=0.00005, kd=0.006025, alpha=0.65, beta=0.3, max_theta=model.maxtheta, conversion="tanh")

# Camera frame capture method
def capture_frame():
    # Capture a frame from the camera
    frame = picam2.capture_array()
    if frame is not None:
        return frame
    else:
        print("Failed to capture image")
        return None

# Main control loop
def control_loop():
    x_t, y_t = (100, 75)  # Target coordinates

    try:
        while True:
            # Capture a frame from the camera
            frame = capture_frame()
            if frame is None:
                continue

            # Process the frame to find ball position (implement your own ball detection logic here)
            # For example, detecting a yellow ball in the center
            x, y = detect_ball_position(frame)

            # Compute desired tilt using PID
            theta, phi = pid.pid((x_t, y_t), (x, y))

            # Move robot based on PID output
            robot.Goto_N_time_spherical(theta, phi, 8.26)

            # Optional: Display frame (for debugging)
            cv2.imshow("Captured Frame", frame)
            cv2.waitKey(1)

    except KeyboardInterrupt:
        print("Control loop interrupted")

    finally:
        picam2.stop()
        cv2.destroyAllWindows()

# Function to detect ball position in the frame (you need to implement this)
def detect_ball_position(frame):
    # For simplicity, this example just returns fixed coordinates
    # You should implement real ball detection logic here
    return 100, 75

# Start control loop
control_loop()