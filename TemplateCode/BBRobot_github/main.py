# Import custom modules
import class_BBRobot      # Robot control (inverse kinematics + servos)
import class_Camera       # Camera + ball detection
import class_PID          # PID controller

# Standard libraries
import time
import threading
import numpy as np


# ------------------- IMAGE BUFFER SETUP -------------------
# Define image size (must match camera output ideally)
height = 480
width = 640
channels = 3

# Create an empty image buffer (black image)
# This will be continuously updated by the camera thread
image = np.zeros((height, width, channels), dtype=np.uint8)


# ------------------- ROBOT + PID SETUP -------------------

# Servo IDs (must match your physical servo IDs)
ids = [1, 2, 3]

# PID parameters
# [Kp, Ki, Kd]
# These values control how aggressively the robot reacts
K_PID = [0.015, 0.0001, 0.0051]

# k = output scaling (movement strength)
k = 1

# a = smoothing factor (alpha)
# 1 = no smoothing, lower = smoother but slower
a = 1

# Create objects
Robot = class_BBRobot.BBrobot(ids)
camera = class_Camera.Camera()
pid = class_PID.PID(K_PID, k, a)


# ------------------- INITIALIZATION -------------------

# Turn on servo torque
Robot.set_up()

# Move robot to safe starting position
Robot.Initialize_posture()

# Store initial height (we keep height constant during control)
pz_ini = Robot.ini_pos[2]
    

# ------------------- PERFORMANCE TRACKING -------------------

frame_count = 0
start_time = time.time()

img_start_time = time.time()
rob_start_time = time.time()

fps = 0       # overall FPS (not used)
img_fps = 0   # camera FPS
rob_fps = 0   # processing FPS


# ------------------- GLOBAL SHARED VARIABLES -------------------

# Ball position and size
x = -1
y = -1
area = -1

# Target position (VERY IMPORTANT)
# This is where you want the ball to go
goal = [100, 0]


# ------------------- THREAD 1: CAMERA CAPTURE -------------------

def get_img():
    """
    Continuously captures images from the camera.
    Runs in a separate thread.
    """
    global image, img_fps, img_start_time

    img_frame_count = 0

    while(1):
        # Capture image from camera
        image = camera.take_pic()
        
        # FPS calculation
        img_frame_count += 1
        if img_frame_count == 100:
            img_end_time = time.time()
            img_elapsed_time = img_end_time - img_start_time

            # Compute frames per second
            img_fps = 100 / img_elapsed_time

            # Reset counters
            img_start_time = img_end_time
            img_frame_count = 0


# ------------------- THREAD 2: IMAGE PROCESSING -------------------

def cont_rob():
    """
    Processes the image to detect the ball.
    Runs in a separate thread.
    """
    global x, y, area, rob_fps, rob_start_time

    rob_frame_count = 0

    while(1):
        # Detect ball in the latest image
        x, y, area = camera.find_ball(image)
        
        # FPS calculation
        rob_frame_count += 1
        if rob_frame_count == 100:
            rob_end_time = time.time()
            rob_elapsed_time = rob_end_time - rob_start_time

            rob_fps = 100 / rob_elapsed_time

            rob_start_time = rob_end_time
            rob_frame_count = 0


# ------------------- MAIN PROGRAM -------------------

try:
    # Create threads
    camera_thread = threading.Thread(target=get_img)
    rob_thread = threading.Thread(target=cont_rob)

    # Start threads
    camera_thread.start()
    rob_thread.start()

    # Main control loop
    while(1):
        # Current detected ball position
        Current_value = [x, y, area]

        # Only run PID if ball is detected
        if x != -1:
            # Compute control output
            theta, phi = pid.compute(goal, Current_value)

            # Keep height constant, only tilt platform
            pos = [theta, phi, pz_ini]

            # Send command to robot
            # 0.01 sec = very fast updates
            Robot.control_t_posture(pos, 0.01)

        # Print performance info
        print(f"img_fps: {img_fps}, rob_fps: {rob_fps}")

finally:
    # Cleanup (VERY IMPORTANT)
    Robot.clean_up()
    camera.clean_up_cam()