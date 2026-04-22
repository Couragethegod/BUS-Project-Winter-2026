import cv2
import numpy as np
import time
from picamera2 import Picamera2
from adafruit_servokit import ServoKit


time.sleep(5)

# ------------------ SERVO SETUP ------------------
kit = ServoKit(channels=16)

SERVO_IDS = [2, 0, 1]
SERVO_MIN = 45
SERVO_MAX = 80
SERVO_NEUTRAL = 60

def clamp(val):
    return max(SERVO_MIN, min(SERVO_MAX, val))


# move smoothly to neutral (60) at startup
TARGET = 60

for step in range(30):
    for i in SERVO_IDS:
        current = kit.servo[i].angle or TARGET
        kit.servo[i].angle = current + (TARGET - current) * 0.1
    time.sleep(0.02)

time.sleep(0.5)

# ------------------ PID ------------------
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

pid_x = PID(0.03, 0.003, 0.02)
pid_y = PID(0.03, 0.003, 0.02)

# ------------------ INVERSE KINEMATICS ------------------
def platform_to_servos(tx, ty):
    """
    tx, ty = tilt values from PID
    Converts tilt to 3 servo angles (120 deg apart)
    """

    angles = []

    # Servo angles in radians (0°, 120°, 240°)
    servo_dirs = [0, 2*np.pi/3, 4*np.pi/3]

    for theta in servo_dirs:
        val = SERVO_NEUTRAL + tx * np.cos(theta) + ty * np.sin(theta)
        angles.append(clamp(val))

    return angles

# ------------------ CAMERA SETUP ------------------
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()

# ------------------ BALL DETECTION ------------------
def detect_ball(frame):
    frame = cv2.GaussianBlur(frame, (5,5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # yellow range
    lower = np.array([20, 100, 100])
    upper = np.array([35, 255, 255])

    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(c)

        if radius > 5:
            return int(x), int(y)

    return None

# ------------------ MAIN LOOP ------------------
prev_time = time.time()

while True:
    frame = picam2.capture_array()
    h, w, _ = frame.shape

    center_x = w // 2
    center_y = h // 2

    ball = detect_ball(frame)

    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    if ball:
        x, y = ball

        # draw ball
        cv2.circle(frame, (x, y), 10, (0, 255, 0), 2)

        # error
        error_x = center_x - x
        error_y = center_y - y

        # PID output = tilt
        tx = pid_x.compute(error_x, dt)
        ty = pid_y.compute(error_y, dt)

        tx = max(-10, min(10, tx))
        ty = max(-10, min(10, ty))

        min_tilt = 1.5

        if 0 < abs(tx) < min_tilt:
            tx = min_tilt * np.sign(tx)

        if 0 < abs(ty) < min_tilt:
            ty = min_tilt * np.sign(ty)

        if abs(error_x) < 10:
            error_x = 0
        if abs(error_y) < 10:
            error_y = 0

        # inverse kinematics
        angles = platform_to_servos(tx, ty)

        # move servos
        for i in range(3):
            kit.servo[SERVO_IDS[i]].angle = angles[i]

    # draw center
    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

    cv2.imshow("Ball Tracking", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):

        # smooth move to 35
        for angle in range(60, 34, -1):
            for i in SERVO_IDS:
                kit.servo[i].angle = angle
            time.sleep(0.02)

        # cut power
        for i in SERVO_IDS:
            kit.servo[i].angle = None

        break

cv2.destroyAllWindows()