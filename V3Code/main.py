import cv2
import numpy as np
import time
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# --- Initialization ---
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Servo settings
SERVO_MIN = 45   # User defined safe min
SERVO_MAX = 80   # User defined safe max
NEUTRAL = 62.5   # Flat platform position

# PID Constants (These will need tuning!)
Kp = 0.15
Ki = 0.01
Kd = 0.05

# State variables for PID
last_error_x = 0
last_error_y = 0
integral_x = 0
integral_y = 0

def set_servo_angle(channel, angle):
    """Maps 0-180 degrees to PCA9685 duty cycle."""
    angle = max(SERVO_MIN, min(SERVO_MAX, angle))
    # Standard MG996R pulse range is roughly 500-2500us
    # Mapping degrees to 16-bit duty cycle (0-65535)
    pulse = int((angle / 180.0) * (2500 - 500) + 500)
    duty_cycle = int((pulse * 65535) / 20000) # 20ms period at 50Hz
    pca.channels[channel].duty_cycle = duty_cycle

def get_ball_pos(frame):
    """Detects teal ball and returns (x, y) coordinates."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Teal color range (Hue is roughly 80-100)
    lower_teal = np.array([75, 100, 50])
    upper_teal = np.array([105, 255, 255])
    
    mask = cv2.bitwise_and(hsv, hsv, mask=cv2.inRange(hsv, lower_teal, upper_teal))
    mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if radius > 10:
            return (int(x), int(y)), mask
    return None, mask

# Initialize Camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

center_target = (160, 120)

try:
    while True:
        ret, frame = cap.read()
        if not ret: break

        ball_data, mask = get_ball_pos(frame)
        
        if ball_data:
            pos, _ = ball_data
            cv2.circle(frame, pos, 10, (0, 255, 0), -1)
            
            # 1. Calculate Error
            error_x = center_target[0] - pos[0]
            error_y = center_target[1] - pos[1]
            
            # 2. PID Calculations
            integral_x += error_x
            integral_y += error_y
            derivative_x = error_x - last_error_x
            derivative_y = error_y - last_error_y
            
            out_x = (Kp * error_x) + (Ki * integral_x) + (Kd * derivative_x)
            out_y = (Kp * error_y) + (Ki * integral_y) + (Kd * derivative_y)
            
            last_error_x = error_x
            last_error_y = error_y
            
            # 3. Inverse Kinematics (Simplified for 120-degree spacing)
            # Servo 1 is at 0 deg, Servo 2 at 120, Servo 3 at 240
            s1_angle = NEUTRAL + out_y
            s2_angle = NEUTRAL - (0.5 * out_y) + (0.866 * out_x)
            s3_angle = NEUTRAL - (0.5 * out_y) - (0.866 * out_x)
            
            set_servo_angle(0, s1_angle)
            set_servo_angle(1, s2_angle)
            set_servo_angle(2, s3_angle)
        else:
            # If ball is lost, return to neutral slowly or stay
            set_servo_angle(0, NEUTRAL)
            set_servo_angle(1, NEUTRAL)
            set_servo_angle(2, NEUTRAL)

        cv2.imshow("Camera Output", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    # Reset servos to neutral on exit
    set_servo_angle(0, NEUTRAL)
    set_servo_angle(1, NEUTRAL)
    set_servo_angle(2, NEUTRAL)