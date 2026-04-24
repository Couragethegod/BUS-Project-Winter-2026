import cv2
import numpy as np
import time
from picamera2 import Picamera2
from adafruit_servokit import ServoKit

# ------------------ SERVO SETUP ------------------
# Initialize the PCA9685 servo driver with 16 channels.
# This allows us to control multiple servos using I2C from the Raspberry Pi.
kit = ServoKit(channels=16)

# These are the channel IDs on the PCA9685 that each servo is connected to.
# The order matters because it maps to our inverse kinematics output.
SERVO_IDS = [2, 0, 1]

# These limits protect the physical system from over-rotation.
# They were experimentally chosen based on the safe tilt range of the platform.
SERVO_MIN = 45
SERVO_MAX = 80

# Neutral position = flat platform (no tilt).
SERVO_NEUTRAL = 60

# Helper function to clamp servo angles within safe bounds.
# Prevents mechanical strain and jitter at extreme positions.
def restric(val):
    return max(SERVO_MIN, min(SERVO_MAX, val))


# ------------------ PID CONTROLLER ------------------
# This class implements a standard PID controller.
# It continuously adjusts output based on error, accumulated error, and rate of change.
class PID:
    def __init__(self, kp, ki, kd):
        # Tuned gains for proportional, integral, and derivative terms.
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Store previous error for derivative calculation.
        self.prev_error = 0

        # Integral term accumulates error over time.
        self.integral = 0

    def compute(self, error, dt):
        # Accumulate integral (area under error curve).
        self.integral += error * dt

        # Derivative = rate of change of error.
        derivative = (error - self.prev_error) / dt if dt > 0 else 0

        # Store current error for next iteration.
        self.prev_error = error

        # Final PID output = weighted sum of all components.
        return self.kp * error + self.ki * self.integral + self.kd * derivative


# Create two independent PID controllers:
# One for X-axis tilt and one for Y-axis tilt.
pid_x = PID(0.03, 0.003, 0.02)
pid_y = PID(0.03, 0.003, 0.02)


# ------------------ INVERSE KINEMATICS ------------------
def platform_to_servos(tx, ty):
    """
    Converts platform tilt (tx, ty) into individual servo angles.

    The platform is supported by 3 servos spaced 120° apart.
    Each servo contributes to tilt depending on its angular position.
    """

    angles = []

    # Define servo directions in radians (0°, 120°, 240°).
    # This models the physical placement of the motors.
    servo_dirs = [0, 2*np.pi/3, 4*np.pi/3]

    for theta in servo_dirs:
        # Combine X and Y tilt using cosine/sine projection.
        # This distributes tilt across the three motors.
        val = SERVO_NEUTRAL + tx * np.cos(theta) + ty * np.sin(theta)

        # Clamp to safe range before sending to hardware.
        angles.append(restric(val))

    return angles


# ------------------ CAMERA SETUP ------------------
# Initialize Raspberry Pi camera using Picamera2.
picam2 = Picamera2()

# Configure resolution and color format for OpenCV compatibility.
picam2.configure(
    picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)}
    )
)

# Start the camera stream.
picam2.start()


# ------------------ BALL DETECTION ------------------
def detect_ball(frame):
    # Apply Gaussian blur to reduce noise and smooth the image.
    frame = cv2.GaussianBlur(frame, (5,5), 0)

    # Convert image to HSV color space for easier color segmentation.
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define HSV range for detecting a yellow ball.
    # These values were tuned experimentally.
    lower = np.array([20, 100, 100])
    upper = np.array([35, 255, 255])

    # Create a binary mask where yellow pixels are white.
    mask = cv2.inRange(hsv, lower, upper)

    # Clean up noise using erosion followed by dilation.
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours (blobs) in the mask.
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Select the largest contour (assumed to be the ball).
        c = max(contours, key=cv2.contourArea)

        # Approximate it as a circle.
        (x, y), radius = cv2.minEnclosingCircle(c)

        # Ignore very small detections (noise).
        if radius > 5:
            return int(x), int(y)

    # Return None if no valid ball is found.
    return None


# ------------------ MAIN CONTROL LOOP ------------------
# Store previous time for calculating delta time (dt).
prev_time = time.time()

while True:
    # Capture a frame from the camera.
    frame = picam2.capture_array()

    # Get frame dimensions.
    h, w, _ = frame.shape

    # Define desired position (center of the platform).
    center_x = w // 2
    center_y = h // 2

    # Detect ball position.
    ball = detect_ball(frame)

    # Compute time difference between frames.
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    if ball:
        x, y = ball

        # Draw detected ball for visualization.
        cv2.circle(frame, (x, y), 10, (0, 255, 0), 2)

        # Calculate error (distance from center).
        error_x = center_x - x
        error_y = center_y - y

        # Use PID controllers to compute required tilt.
        tx = pid_x.compute(error_x, dt)
        ty = pid_y.compute(error_y, dt)

        # Limit tilt output to prevent extreme motion.
        tx = max(-10, min(10, tx))
        ty = max(-10, min(10, ty))

        # Minimum tilt helps overcome static friction (ball sticking).
        min_tilt = 1.5

        if 0 < abs(tx) < min_tilt:
            tx = min_tilt * np.sign(tx)

        if 0 < abs(ty) < min_tilt:
            ty = min_tilt * np.sign(ty)

        # Deadband: ignore very small errors to reduce jitter.
        if abs(error_x) < 10:
            error_x = 0
        if abs(error_y) < 10:
            error_y = 0

        # Convert tilt into servo angles.
        angles = platform_to_servos(tx, ty)

        # Send angles to each servo motor.
        for i in range(3):
            kit.servo[SERVO_IDS[i]].angle = angles[i]

    # Draw center point (target position).
    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

    # Display the camera feed.
    cv2.imshow("Ball Tracking", frame)

    # Check for key press.
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):

        # Smoothly return platform to neutral position before shutdown.
        for angle in range(60, 34, -1):
            for i in SERVO_IDS:
                kit.servo[i].angle = angle
            time.sleep(0.04)

        # Disable servo signals to cut power and prevent jitter.
        for i in SERVO_IDS:
            kit.servo[i].angle = None

        break

# Close all OpenCV windows.
cv2.destroyAllWindows()