import threading
import time
import cv2
import math
import numpy as np

# Custom modules
from camera import Camera
from controller import RobotController
from robotKinematics import RobotKinematics
from PID import PIDcontroller


# ================================
# SHARED STATE BETWEEN THREADS
# ================================

# Latest camera frame (shared between capture + processing threads)
latest_frame = np.zeros((200, 150, 3), dtype=np.uint8)

# Lock prevents race conditions when reading/writing frames
lock = threading.Lock()

# Global flag to stop threads safely
running = True


# ================================
# PID TUNING PARAMETERS
# ================================

# These directly control robot responsiveness:
# - kp: how aggressively robot reacts to error
# - ki: corrects steady-state error (drift)
# - kd: dampens motion (prevents overshoot)

# ⚠️ You overwrote values multiple times → only last set matters
kp = 0.0063      # Lower = smoother but slower response
ki = 0.00005     # Too high → oscillation / drift
kd = 0.006025    # Higher = more damping

# ⚠️ These look like smoothing/filtering parameters (depends on PID implementation)
alpha = 0.65
beta = 0.3


# ================================
# INITIALIZE HARDWARE + MODEL
# ================================

cam = Camera()

# Kinematics model (math)
model = RobotKinematics()

# Controller (hardware interface)
# ⚠️ You're passing geometry AGAIN → redundant if already in model
robot = RobotController(model, model.lp, model.l1, model.l2, model.lb)

# PID controller
PID = PIDcontroller(
    kp, ki, kd,
    alpha, beta,
    max_theta=model.maxtheta,   # Limits tilt angle
    conversion="tanh"           # Likely smooths output (good for stability)
)


# ================================
# INITIAL BALL POSITION (DEFAULT)
# ================================

# These are fallback values before camera detects anything
x, y = 100, 75


# ================================
# THREAD 1: CAMERA CAPTURE
# ================================

def capture():
    global latest_frame, running

    while running:
        frame = cam.take_picture()
        with lock:
            latest_frame = frame


# ================================
# THREAD 2: IMAGE PROCESSING + CONTROL
# ================================

def process():
    """
    Main control loop:
    - Reads latest frame
    - Detects ball position
    - Runs PID → computes tilt
    - Sends commands to robot

    hz = 50 → 50Hz control loop (good target)
    """

    hz = 50

    global latest_frame, x, y

    while True:
        # Copy frame safely
        with lock:
            if latest_frame is None:
                continue
            frame_copy = latest_frame.copy()

        loop_start = time.perf_counter()

        # === COMPUTER VISION ===
        # Detect ball position (pixel coordinates)
        x, y = cam.coordinate(frame_copy)

        # === TARGET POSITION ===
        # You want the ball centered
        x_t, y_t = (100, 75)

        # === CONTROL STEP ===
        update_robot_pos(robot, model, PID, x_t, y_t, x, y)

        # Optional visualization (slow → affects performance)
        # cam.display_draw(frame_copy, (x,y))

        # === TIMING CONTROL ===
        elapsed = time.perf_counter() - loop_start
        sleep_time = (1 / hz) - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)


# ================================
# CONTROL HELPER FUNCTION
# ================================

def update_robot_pos(robotcontroller, robotkinematics, pidcontroller,
                     x_t, y_t, x, y):
    """
    Converts position error → robot movement.

    Steps:
    1. PID computes desired tilt (theta, phi)
    2. Robot controller executes movement

    USER TUNING:
    - This is where you'd clamp max tilt
    - Or smooth movement
    """

    # PID output → desired platform tilt
    theta, phi = pidcontroller.pid((x_t, y_t), (x, y))

    # ⚠️ No safety limits applied here → dangerous
    # Should clamp theta based on robotkinematics.maxtheta

    # Send command to robot
    robotcontroller.Goto_N_time_spherical(theta, phi, 8.26)

    # ⚠️ Hardcoded height (8.26)
    # This should match your calibrated working height


# ================================
# OPTIONAL: SEPARATE PID LOOP (NOT USED)
# ================================

def pid_loop():
    """
    Alternative PID loop (currently unused).

    This would decouple:
    - vision (process thread)
    - control (this thread)

    RIGHT NOW:
    You're doing both in one loop → simpler but less stable
    """

    hz = 30

    while running:
        loop_start = time.perf_counter()

        x_t, y_t = (100, 75)

        update_robot_pos(robot, model, PID, x_t, y_t, x, y)

        elapsed = time.perf_counter() - loop_start
        sleep_time = (1 / hz) - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)


# ================================
# START THREADS
# ================================

threading.Thread(target=capture, daemon=True).start()
threading.Thread(target=process, daemon=True).start()

# Give camera time to initialize
time.sleep(2)

#threading.Thread(target=pid_loop).start()


# ================================
# MAIN LOOP (KEEP PROGRAM ALIVE)
# ================================

try:
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    running = False
    print("Exiting...")