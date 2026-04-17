#!/usr/bin/env python3
"""
Smart Ball Balancer Robot
Raspberry Pi 4B · Pi Camera 2 · PCA9685 · 3x MG996R Servos
"""

import cv2
import numpy as np
import time
import math
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# ── Config ─────────────────────────────────────────────────────────────────────

SERVO_CHANNELS  = [0, 2, 1]
SERVO_FREQ      = 50          # Hz

ANGLE_MIN = 45.0
ANGLE_MAX = 80.0
ANGLE_MID = (ANGLE_MIN + ANGLE_MAX) / 2.0   # 62.5° = flat

PULSE_MIN_US = 500    # µs → 0°
PULSE_MAX_US = 2500   # µs → 360°

FRAME_W, FRAME_H = 640, 480
CX, CY = FRAME_W // 2, FRAME_H // 2

# Teal ball HSV range
TEAL_LO = np.array([90,  100,  100])
TEAL_HI = np.array([100, 255, 255])

# PID gains — tune KP first, then KD
KP, KI, KD = 0.04, 0.0005, 0.018

# Leg azimuths: 3 servos spaced 120° apart
LEG_PHI = [math.radians(a) for a in (0, 120, 240)]

# ── Servo helpers ───────────────────────────────────────────────────────────────

def angle_to_pwm(deg):
    deg   = float(np.clip(deg, ANGLE_MIN, ANGLE_MAX))
    us    = PULSE_MIN_US + (deg / 360.0) * (PULSE_MAX_US - PULSE_MIN_US)
    count = int((us / 20000.0) * 4096)   # 50 Hz → 20 ms period
    return int(np.clip(count, 0, 4095)) << 4   # 12-bit → 16-bit

def set_servos(pca, angles):
    for ch, ang in zip(SERVO_CHANNELS, angles):
        pca.channels[ch].duty_cycle = angle_to_pwm(ang)

# ── Inverse kinematics ──────────────────────────────────────────────────────────

def ik(tilt_x, tilt_y):
    """Map tilt command (–1…+1) to 3 servo angles within safe range."""
    half = (ANGLE_MAX - ANGLE_MIN) / 2.0
    return [
        float(np.clip(ANGLE_MID + (tilt_x * math.cos(p) + tilt_y * math.sin(p)) * half,
                      ANGLE_MIN, ANGLE_MAX))
        for p in LEG_PHI
    ]

# ── PID ─────────────────────────────────────────────────────────────────────────

class PID2D:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral   = np.zeros(2)
        self.prev_error = np.zeros(2)
        self.prev_t     = time.time()

    def reset(self):
        self.integral[:] = 0; self.prev_error[:] = 0

    def update(self, error):
        now = time.time(); dt = max(now - self.prev_t, 1e-4)
        self.integral  += error * dt
        deriv           = (error - self.prev_error) / dt
        out             = self.kp * error + self.ki * self.integral + self.kd * deriv
        self.prev_error = error.copy(); self.prev_t = now
        return np.clip(out, -1.0, 1.0)

# ── Ball detection ──────────────────────────────────────────────────────────────

def detect_ball(frame):
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, TEAL_LO, TEAL_HI)
    mask = cv2.erode(mask,  None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < 300:
        return None
    (x, y), r = cv2.minEnclosingCircle(c)
    return int(x), int(y), int(r)

# ── Main ────────────────────────────────────────────────────────────────────────

def main():
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = SERVO_FREQ
    set_servos(pca, [ANGLE_MID] * 3)
    time.sleep(0.5)

    pid = PID2D(KP, KI, KD)
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    angles = [ANGLE_MID] * 3
    t_prev = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            frame = cv2.flip(frame, -1)   # camera is upside-down underneath

            ball = detect_ball(frame)

            if ball:
                bx, by, br = ball
                ex = (bx - CX) / CX
                ey = (by - CY) / CY
                tilt   = pid.update(np.array([ex, ey]))
                angles = ik(tilt[0], tilt[1])

                # Draw ball circle + line to centre
                cv2.circle(frame, (bx, by), br, (0, 220, 160), 2)
                cv2.circle(frame, (bx, by),  4, (0, 255, 200), -1)
                cv2.line(frame,   (bx, by), (CX, CY), (0, 200, 255), 1)
            else:
                pid.reset()
                angles = [ANGLE_MID] * 3

            set_servos(pca, angles)

            # Centre crosshair
            cv2.drawMarker(frame, (CX, CY), (255, 255, 255),
                           cv2.MARKER_CROSS, 16, 1)

            # FPS & servo readout
            now = time.time()
            fps = 1.0 / max(now - t_prev, 1e-4); t_prev = now
            cv2.putText(frame, f"FPS {fps:.0f}", (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 255, 220), 1)
            cv2.putText(frame,
                        f"S0:{angles[0]:.1f}  S1:{angles[1]:.1f}  S2:{angles[2]:.1f}",
                        (8, FRAME_H - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 220, 255), 1)

            cv2.imshow("Ball Balancer", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        set_servos(pca, [ANGLE_MID] * 3)
        cap.release()
        cv2.destroyAllWindows()
        pca.deinit()

if __name__ == "__main__":
    main()