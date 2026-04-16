import time
import cv2
import numpy as np
import math
from adafruit_pca9685 import PCA9685
import busio
import board

# Define the PID controller
class PIDcontroller:
    def __init__(self, kp, ki, kd, alpha, beta, max_theta, conversion="tanh"):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.alpha = alpha
        self.beta = beta
        self.max_theta = max_theta
        self.conversion = conversion
        self.prev_error = 0
        self.integral = 0

    def pid(self, target, current):
        error_x = target[0] - current[0]
        error_y = target[1] - current[1]

        self.integral += error_x + error_y
        derivative = error_x + error_y - self.prev_error
        self.prev_error = error_x + error_y

        # Calculate the PID output
        output = self.kp * (error_x + error_y) + self.ki * self.integral + self.kd * derivative
        output = np.clip(output, -self.max_theta, self.max_theta)

        # Convert to desired angles (this can be adjusted based on your model)
        theta = output
        phi = output

        return theta, phi


# Setup the camera (using OpenCV)
def capture_frame():
    # Open the first camera device
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Camera not accessible!")
        return None

    ret, frame = cap.read()
    cap.release()

    if ret:
        return frame
    else:
        print("Error: Failed to capture image")
        return None


# Set up PCA9685 for servo control
def setup_servos():
    # Create I2C bus and initialize PCA9685
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50

    return pca


# Convert angle to PWM (servo control)
def angle_to_pwm(angle, min_angle=45, max_angle=80):
    # Map angle (45 to 80 degrees) to PWM (0 to 360)
    pwm_range = 360
    min_pwm = 0
    max_pwm = pwm_range

    return int(min_pwm + (angle - min_angle) * (max_pwm - min_pwm) / (max_angle - min_angle))


# Move servos to given angles
def move_servos(pca, servo_1_angle, servo_2_angle, servo_3_angle):
    pca.channels[0].duty_cycle = angle_to_pwm(servo_1_angle) << 4
    pca.channels[1].duty_cycle = angle_to_pwm(servo_2_angle) << 4
    pca.channels[2].duty_cycle = angle_to_pwm(servo_3_angle) << 4
    time.sleep(0.1)


# Ball detection logic (you can refine this)
def detect_ball_position(frame):
    # Convert to HSV (helps with color detection)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define yellow color range in HSV
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    # Mask the image to get yellow areas
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # Find contours of the yellow areas
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Assume the largest contour is the ball
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)

        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return cx, cy

    return 0, 0  # If no ball is detected


# Main function for controlling the robot
def main():
    # Set up PID controller
    pid = PIDcontroller(kp=0.0063, ki=0.00005, kd=0.006025, alpha=0.65, beta=0.3, max_theta=20)

    # Set up the servo controller (PCA9685)
    pca = setup_servos()

    # Set servos to 55 degrees initially
    move_servos(pca, 55, 55, 55)

    target_x, target_y = 100, 75  # Target position of the ball (center of platform)

    try:
        while True:
            # Capture the frame from the camera
            frame = capture_frame()
            if frame is None:
                continue

            # Detect ball position
            x, y = detect_ball_position(frame)
            print(f"Ball Position: x={x}, y={y}")

            # Compute PID control
            theta, phi = pid.pid((target_x, target_y), (x, y))

            # Move servos based on PID output
            move_servos(pca, theta + 55, phi + 55, 55)  # Adjusting servo angles (55 min, 80 max)

            # Optional: Show the frame with the detected ball
            cv2.circle(frame, (x, y), 10, (0, 0, 255), -1)  # Draw a red circle at the ball's position
            cv2.imshow("Ball Detection", frame)

            # Break if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()