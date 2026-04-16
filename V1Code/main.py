import cv2
import numpy as np
import time
from picamera2 import Picamera2
from adafruit_servokit import ServoKit
from collections import deque

# ------------------ SERVO SETUP ------------------
kit = ServoKit(channels=16)

SERVO_MIN = 45
SERVO_MAX = 80
SERVO_NEUTRAL = 62  # Neutral position, adjust to your platform level

def clamp(val):
    """Ensure servo values stay within the valid range."""
    return max(SERVO_MIN, min(SERVO_MAX, val))

def set_servos(a1, a2, a3):
    """Set servo angles."""
    kit.servo[0].angle = clamp(a1)
    kit.servo[1].angle = clamp(a2)
    kit.servo[2].angle = clamp(a3)

# ------------------ CAMERA SETUP ------------------
picam2 = Picamera2()

# Configure the camera for preview mode
config = picam2.create_preview_configuration(main={"size": (320, 240)})
picam2.configure(config)

# Start the camera
picam2.start()

# ------------------ PID PARAMETERS ------------------
kp = 0.05
kd = 0.02
prev_err_x = 0
prev_err_y = 0

# ------------------ TARGET COORDINATES ------------------
TARGET_X = 160  # Center X (horizontal center of the frame)
TARGET_Y = 120  # Center Y (vertical center of the frame)

# ------------------ YELLOW COLOR DETECTION RANGE ------------------
lower_yellow = np.array([20, 100, 100])  # Lower bound of yellow in HSV
upper_yellow = np.array([30, 255, 255])  # Upper bound of yellow in HSV

# ------------------ CAMERA CLASS ------------------
class Camera:
    def __init__(self, resolution=(1640, 1232), format="RGB888"):
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": resolution, "format": format}, controls={"FrameDurationLimits": (8333, 8333)})
        self.picam2.configure(config)

        self.lower_black = np.array([0, 0, 0])
        self.upper_black = np.array([180, 255, 50])
        self.gray_threshold = 60
        self.queue = deque(maxlen=16)
        self.queue.append((100, 75))  # Start with a default position

        self.picam2.start()

    def take_picture(self):
        """Capture an image frame and return it."""
        try:
            # Capture frame using Picamera2
            frame = self.picam2.capture_array()
            scale = 200.0 / frame.shape[1]
            frame_resized = cv2.resize(frame, (200, int(frame.shape[0] * scale)))
            return frame_resized
        except Exception as e:
            print(f"Error capturing frame: {e}")
            return None

    def display(self, image, window_name="Camera Output"):
        """Display the image in a window."""
        if image is not None:
            cv2.imshow(window_name, image)
            cv2.waitKey(1)

    def display_draw(self, image, center, window_name="Tracked Output"):
        """Display the frame with drawn tracking circles."""
        x, y = center
        cv2.line(image, (x - 10, y), (x + 10, y), (0, 0, 255), 2)
        cv2.line(image, (x, y - 10), (x, y + 10), (0, 0, 255), 2)
        self.display(image, window_name)

    def terminate(self):
        """Stop and release the camera."""
        self.picam2.stop()
        self.picam2.close()
        cv2.destroyAllWindows()

    def coordinate(self, image):
        """Process the image to track the object."""
        frame_blurred = cv2.GaussianBlur(image, (3, 3), 0)
        
        frame_hsv = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2HSV)
        frame_gray = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2GRAY)

        mask_hsv = cv2.inRange(frame_hsv, self.lower_black, self.upper_black)
        mask_gray = cv2.threshold(frame_gray, self.gray_threshold, 255, cv2.THRESH_BINARY_INV)[1]
        mask_combined = cv2.bitwise_or(mask_hsv, mask_gray)

        mask_eroded = cv2.erode(mask_combined, None, iterations=1)
        mask_dilated = cv2.dilate(mask_eroded, None, iterations=1)

        valid_detections = []
        contours, _ = cv2.findContours(mask_dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            radius = int(radius)

            if radius < 5 or radius > 100:
                continue

            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
            circularity = (4 * np.pi * area) / (perimeter ** 2)
            if circularity < 0.6:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            valid_detections.append((area, (int(x + w / 2), int(y + h / 2))))

        if valid_detections:
            best_center = max(valid_detections, key=lambda item: item[0])[1]  
            self.queue.append(best_center) 
        else:
            if len(self.queue) >= 5 and self.queue[-1] == self.queue[-2] == self.queue[-3] == self.queue[-4] == self.queue[-5]:
                self.queue.append((100, 75)) 
            else:
                self.queue.append(self.queue[-1])

        return self.queue[-1]

# ------------------ MAIN LOOP ------------------
cam = Camera()

try:
    while True:
        # Capture image
        img = cam.take_picture()

        if img is None:
            print("No image captured. Skipping frame.")
            continue

        # Get the center of the object (ball position)
        c = cam.coordinate(img)

        # Debug: Display the tracking result
        cam.display_draw(img, c)

        # PID control for servo movement
        x, y = c
        err_x = x - TARGET_X  # Error in X direction
        err_y = y - TARGET_Y  # Error in Y direction

        d_err_x = err_x - prev_err_x
        d_err_y = err_y - prev_err_y

        control_x = kp * err_x + kd * d_err_x
        control_y = kp * err_y + kd * d_err_y

        prev_err_x = err_x
        prev_err_y = err_y

        # Servo mixing
        s1 = SERVO_NEUTRAL + control_x
        s2 = SERVO_NEUTRAL - control_x / 2 + control_y
        s3 = SERVO_NEUTRAL - control_x / 2 - control_y

        # Set servo positions
        set_servos(s1, s2, s3)

        print(f"Ball Position: {c}")

        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cam.terminate()