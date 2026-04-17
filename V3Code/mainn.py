#!/usr/bin/env python3
"""
Smart Ball Balancer Robot
Hardware: Raspberry Pi 4B, Pi Camera 2, PCA9685, 3x MG996R Servos
Platform: Clear acrylic, camera mounted underneath
"""

import cv2
import numpy as np
import time
import math
import threading
from flask import Flask, Response, jsonify
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# ── Configuration ──────────────────────────────────────────────────────────────

# Servo channels on PCA9685
SERVO_CHANNELS = [0, 1, 2]          # 3 servos at 120° apart
SERVO_FREQ     = 50                  # Hz

# Safe angle range (degrees) — mapped from your 45–80° safe zone
ANGLE_MIN = 45
ANGLE_MAX = 80
ANGLE_MID = (ANGLE_MIN + ANGLE_MAX) / 2   # 62.5° neutral/flat

# PWM pulse widths (µs) for MG996R over 0–360° range
PULSE_MIN = 500    # µs  → 0°
PULSE_MAX = 2500   # µs  → 360°

# Camera / platform
FRAME_W, FRAME_H = 640, 480
CENTER_X = FRAME_W // 2
CENTER_Y = FRAME_H // 2
MAX_DIST = math.hypot(CENTER_X, CENTER_Y)

# HSV range for teal ball
TEAL_HSV_LO = np.array([80,  80,  60])
TEAL_HSV_HI = np.array([100, 255, 255])

# PID gains  (tune these for your platform)
KP, KI, KD = 0.04, 0.0005, 0.018

# Servo geometry — legs at 0°, 120°, 240°
LEG_ANGLES_RAD = [math.radians(a) for a in (0, 120, 240)]

# ── Servo helpers ──────────────────────────────────────────────────────────────

def angle_to_pwm(angle_deg: float) -> int:
    """Convert angle (0–360°) to 12-bit PCA9685 PWM value."""
    angle_deg = float(np.clip(angle_deg, ANGLE_MIN, ANGLE_MAX))
    pulse_us   = PULSE_MIN + (angle_deg / 360.0) * (PULSE_MAX - PULSE_MIN)
    # PCA9685 uses 12-bit counts; period = 1/50 Hz = 20 000 µs
    pwm_count  = int((pulse_us / 20000.0) * 4096)
    return np.clip(pwm_count, 0, 4095)


def set_servo(pca, channel: int, angle_deg: float):
    pca.channels[channel].duty_cycle = angle_to_pwm(angle_deg) << 4  # 12→16 bit


# ── Inverse Kinematics ─────────────────────────────────────────────────────────

def ik_platform(tilt_x: float, tilt_y: float) -> list[float]:
    """
    Convert desired platform tilt (tilt_x, tilt_y in normalised –1…+1)
    to individual servo angles.

    Each leg i has a fixed azimuth φ_i. Its required height change is the
    dot product of the tilt vector with that leg's unit direction.
    Positive tilt_x tilts the platform right (ball rolls left to centre).
    """
    half_range = (ANGLE_MAX - ANGLE_MIN) / 2.0
    angles = []
    for phi in LEG_ANGLES_RAD:
        # Height contribution from tilt at this leg's position
        delta = tilt_x * math.cos(phi) + tilt_y * math.sin(phi)
        angle = ANGLE_MID + delta * half_range
        angles.append(float(np.clip(angle, ANGLE_MIN, ANGLE_MAX)))
    return angles


# ── PID Controller ─────────────────────────────────────────────────────────────

class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = np.zeros(2)
        self.prev_error = np.zeros(2)
        self.prev_time = time.time()

    def reset(self):
        self.integral[:] = 0
        self.prev_error[:] = 0

    def update(self, error: np.ndarray) -> np.ndarray:
        now = time.time()
        dt  = max(now - self.prev_time, 1e-4)
        self.integral   += error * dt
        derivative       = (error - self.prev_error) / dt
        output           = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error  = error.copy()
        self.prev_time   = now
        return np.clip(output, -1.0, 1.0)


# ── Ball Detection ─────────────────────────────────────────────────────────────

def detect_ball(frame: np.ndarray):
    """Return (cx, cy, radius) or None if no ball found."""
    hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask   = cv2.inRange(hsv, TEAL_HSV_LO, TEAL_HSV_HI)
    mask   = cv2.erode(mask,  None, iterations=2)
    mask   = cv2.dilate(mask, None, iterations=2)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < 200:
        return None
    ((x, y), r) = cv2.minEnclosingCircle(c)
    return int(x), int(y), int(r)


# ── Shared state (main ↔ Flask) ────────────────────────────────────────────────

state = {
    "ball_x": None, "ball_y": None,
    "error_x": 0.0, "error_y": 0.0,
    "servo_angles": [ANGLE_MID] * 3,
    "fps": 0.0,
    "frame": None,
    "lock": threading.Lock()
}

# ── Flask server for live feed & telemetry ─────────────────────────────────────

app = Flask(__name__)

def gen_frames():
    while True:
        with state["lock"]:
            frame = state["frame"]
        if frame is None:
            time.sleep(0.02)
            continue
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')

@app.route('/video')
def video():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/telemetry')
def telemetry():
    with state["lock"]:
        return jsonify({
            "ball_x": state["ball_x"],
            "ball_y": state["ball_y"],
            "error_x": round(state["error_x"], 3),
            "error_y": round(state["error_y"], 3),
            "servo_angles": [round(a, 1) for a in state["servo_angles"]],
            "fps": round(state["fps"], 1)
        })

@app.route('/')
def index():
    return open('/home/pi/ball_balancer_dashboard.html').read()

# ── Main control loop ──────────────────────────────────────────────────────────

def main():
    # Init I2C & PCA9685
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = SERVO_FREQ

    # Centre all servos
    for ch in SERVO_CHANNELS:
        set_servo(pca, ch, ANGLE_MID)
    time.sleep(0.5)

    pid = PID(KP, KI, KD)
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Start Flask in background
    flask_thread = threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=5000, threaded=True), daemon=True)
    flask_thread.start()
    print("Dashboard → http://<pi-ip>:5000")

    t_prev = time.time()
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            # Flip because camera is mounted underneath (upside-down)
            frame = cv2.flip(frame, -1)

            ball = detect_ball(frame)
            angles = [ANGLE_MID] * 3

            if ball:
                bx, by, br = ball
                # Normalised error  –1…+1
                ex = (bx - CENTER_X) / CENTER_X
                ey = (by - CENTER_Y) / CENTER_Y
                tilt = pid.update(np.array([ex, ey]))
                angles = ik_platform(tilt[0], tilt[1])

                # Annotate frame
                cv2.circle(frame, (bx, by), br, (0, 220, 180), 2)
                cv2.circle(frame, (bx, by), 4,  (0, 255, 255), -1)
                cv2.line(frame, (bx, by), (CENTER_X, CENTER_Y), (0, 180, 255), 1)
            else:
                pid.reset()

            # Drive servos
            for i, ch in enumerate(SERVO_CHANNELS):
                set_servo(pca, ch, angles[i])

            # HUD
            cv2.circle(frame, (CENTER_X, CENTER_Y), 6, (255, 255, 255), -1)
            now = time.time(); fps = 1.0 / max(now - t_prev, 1e-4); t_prev = now
            cv2.putText(frame, f"FPS {fps:.1f}", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 255, 200), 1)
            cv2.putText(frame, f"S0:{angles[0]:.0f} S1:{angles[1]:.0f} S2:{angles[2]:.0f}",
                        (10, FRAME_H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            with state["lock"]:
                state.update({
                    "ball_x": ball[0] if ball else None,
                    "ball_y": ball[1] if ball else None,
                    "error_x": float(ex) if ball else 0.0,
                    "error_y": float(ey) if ball else 0.0,
                    "servo_angles": angles,
                    "fps": fps,
                    "frame": frame.copy()
                })

    except KeyboardInterrupt:
        print("\nShutting down…")
    finally:
        for ch in SERVO_CHANNELS:
            set_servo(pca, ch, ANGLE_MID)
        cap.release()
        pca.deinit()


if __name__ == '__main__':
    main()