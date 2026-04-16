import cv2
import numpy as np
from picamera2 import Picamera2
from adafruit_pca9685 import PCA9685
import board
import busio
import time
from collections import deque

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
    
    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class BallBalancer:
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50
        
        self.servo_channels = [0, 1, 2]
        self.min_angle = 45
        self.max_angle = 80
        self.center_angle = 62.5
        
        self.pid_x = PIDController(kp=0.05, ki=0.01, kd=0.02)
        self.pid_y = PIDController(kp=0.05, ki=0.01, kd=0.02)
        
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"format": 'RGB888', "size": (640, 480)}
        ))
        self.picam2.start()
        
        self.yellow_lower = np.array([20, 100, 100])
        self.yellow_upper = np.array([30, 255, 255])
        
        self.last_time = time.time()
        self.ball_history = deque(maxlen=5)
    
    def angle_to_pwm(self, angle):
        angle = max(self.min_angle, min(self.max_angle, angle))
        pulse_width = (angle / 180) * 2000 + 1000
        return int((pulse_width / 20000) * 65535)
    
    def detect_ball(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy)
        return None
    
    def update_servos(self, ball_pos):
        if not ball_pos:
            return
        
        frame_center_x, frame_center_y = 320, 240
        error_x = ball_pos[0] - frame_center_x
        error_y = ball_pos[1] - frame_center_y
        
        dt = time.time() - self.last_time
        self.last_time = time.time()
        
        servo_x_adjustment = self.pid_x.update(error_x / 320, dt)
        servo_y_adjustment = self.pid_y.update(error_y / 240, dt)
        
        angle_x = self.center_angle + servo_x_adjustment
        angle_y = self.center_angle + servo_y_adjustment
        angle_z = self.center_angle
        
        self.pca.channels[self.servo_channels[0]].duty_cycle = self.angle_to_pwm(angle_x)
        self.pca.channels[self.servo_channels[1]].duty_cycle = self.angle_to_pwm(angle_y)
        self.pca.channels[self.servo_channels[2]].duty_cycle = self.angle_to_pwm(angle_z)
    
    def run(self):
        try:
            while True:
                frame = self.picam2.capture_array()
                ball_pos = self.detect_ball(frame)
                self.update_servos(ball_pos)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.picam2.stop()

if __name__ == "__main__":
    balancer = BallBalancer()
    balancer.run()