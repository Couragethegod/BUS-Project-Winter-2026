import cv2
import numpy as np
import time

from picamera2 import Picamera2
from adafruit_servokit import ServoKit

kit = ServoKit(channels=3)

SERVO_MIN = 45
SERVO_MAX = 80
SERVO_MID = 65