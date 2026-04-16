import time
import cv2
import numpy as np
from picamera2 import PiCamera2
from adafruit_servokit import ServoKit

# --- CONFIGURATION ---
# Initialize PCA9685 (16 channels)
kit = ServoKit(channels=16)

# Servo Limits (per user: 45 to 80 degrees)
SERVO_MIN = 45
SERVO_MAX = 80
SERVO_MID = 62.5  # The "Level" position

# PID Gains (Tune these!)
# Kp: How much to tilt based on distance from center
# Kd: how much to "counter-tilt" based on ball speed
Kp = 0.08 
Kd = 0.03

# Global variables for PID
prev_error_x = 0
prev_error_y = 0

# --- CAMERA SETUP ---
picam2 = PiCamera2()
# Using a lower resolution for faster processing/lower latency
config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
picam2.configure(config)
picam2.start()

def move_servos(tilt_x, tilt_y):
    """
    Translates X/Y tilt into 3 servo angles for 120-degree spacing.
    tilt_x/y are the PID outputs.
    """
    # 120-degree geometry mixing logic
    # Servo 0: Rear center
    # Servo 1: Front right
    # Servo 2: Front left
    s0 = SERVO_MID + tilt_y
    s1 = SERVO_MID - (0.5 * tilt_y) + (0.866 * tilt_x)
    s2 = SERVO_MID - (0.5 * tilt_y) - (0.866 * tilt_x)

    # Apply hardware limits and update servos
    kit.servo[0].angle = max(SERVO_MIN, min(SERVO_MAX, s0))
    kit.servo[1].angle = max(SERVO_MIN, min(SERVO_MAX, s1))
    kit.servo[2].angle = max(SERVO_MIN, min(SERVO_MAX, s2))

try:
    print("Leveling platform...")
    move_servos(0, 0)
    time.sleep(2)

    while True:
        # 1. Capture frame
        frame = picam2.capture_array()
        
        # 2. Process for Yellow (HSV Space)
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        # Typical Yellow Range (adjust if lighting is weird)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest yellow object
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            if radius > 10:
                # 3. PID Logic
                # Center of 640x480 is 320, 240
                error_x = x - 320
                error_y = y - 240

                # Calculate change in error (Derivative)
                dx = error_x - prev_error_x
                dy = error_y - prev_error_y

                # Final Tilt Output
                # Note: Because camera is UNDERNEATH, you may need to 
                # flip the signs (e.g., -error_x) if it tilts the wrong way.
                output_x = (error_x * Kp) + (dx * Kd)
                output_y = (error_y * Kp) + (dy * Kd)

                move_servos(output_x, output_y)

                prev_error_x = error_x
                prev_error_y = error_y
        else:
            # Ball lost: Return to level
            move_servos(0, 0)

except KeyboardInterrupt:
    print("Shutting down...")
finally:
    move_servos(0, 0)
    picam2.stop()