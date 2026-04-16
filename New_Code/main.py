import time
import cv2
import numpy as np
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
from PID import PIDcontroller  # Assuming you have a PID controller class or use a library like simple-pid

# Setup PCA9685 with 16 channels
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Servo control setup
servo_kit = ServoKit(channels=16)

# Servo channel mapping (Assuming servos are connected to channel 0, 1, 2)
servo_channels = [servo_kit.servo[0], servo_kit.servo[1], servo_kit.servo[2]]

# Servo limits (degrees)
SERVO_MIN = 55
SERVO_MAX = 80

# Camera setup (Pi Camera)
cap = cv2.VideoCapture(0)

# Ball color detection range (in HSV)
LOWER_YELLOW = np.array([20, 100, 100])
UPPER_YELLOW = np.array([30, 255, 255])

# PID Tuning Parameters
kp = 0.1
ki = 0.001
kd = 0.01

# PID controller setup (for both X and Y axes)
pid_x = PIDcontroller(kp, ki, kd)
pid_y = PIDcontroller(kp, ki, kd)

# Ball detection method
def detect_ball(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
    blurred = cv2.GaussianBlur(mask, (15, 15), 0)
    contours, _ = cv2.findContours(blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Get the largest contour (ball)
        largest_contour = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

        # Return the center of the ball
        return int(x), int(y)
    
    # If no ball detected, return previous position or set to None
    return None, None

# Move servo to a target position
def move_servo_to_position(servo_index, angle):
    # Apply strict clamping
    if angle < SERVO_MIN:
        angle = SERVO_MIN
    elif angle > SERVO_MAX:
        angle = SERVO_MAX
    servo_channels[servo_index].angle = angle

# Initialize servos to 55° at startup
def initialize_servos():
    print("Initializing servos to 55°...")
    for i in range(3):
        move_servo_to_position(i, 55)
    time.sleep(1)  # Wait for servos to settle

# Main control loop with PID
def control_loop():
    try:
        while True:
            # Capture frame from camera
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture image")
                break

            # Ball detection
            x, y = detect_ball(frame)

            if x is not None and y is not None:
                print(f"Ball detected at: {x}, {y}")

                # Get frame dimensions
                frame_width = frame.shape[1]
                frame_height = frame.shape[0]
                target_x = frame_width // 2
                target_y = frame_height // 2

                # Calculate the error in X and Y directions
                error_x = x - target_x
                error_y = y - target_y

                # Use the PID controllers for X and Y to get the desired angles
                servo1_angle = pid_x.update(error_x)  # X-axis control
                servo2_angle = pid_y.update(error_y)  # Y-axis control

                # Apply servo limits
                servo1_angle = max(SERVO_MIN, min(SERVO_MAX, 55 + servo1_angle))
                servo2_angle = max(SERVO_MIN, min(SERVO_MAX, 55 + servo2_angle))

                # Move the servos based on PID output
                move_servo_to_position(0, servo1_angle)  # X-axis servo
                move_servo_to_position(1, servo2_angle)  # Y-axis servo

            # Display the frame for debugging
            cv2.imshow("Frame", frame)

            # Exit on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Control loop interrupted")

    finally:
        # Release resources
        cap.release()
        cv2.destroyAllWindows()

# Startup sequence
initialize_servos()

# Start the control loop
control_loop()