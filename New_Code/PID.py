class PIDcontroller:
    def __init__(self, kp, ki, kd, windup_limit=10):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.windup_limit = windup_limit
        self.prev_error = 0
        self.integral = 0

    def update(self, error):
        # Integral windup prevention
        self.integral += error
        if self.integral > self.windup_limit:
            self.integral = self.windup_limit
        elif self.integral < -self.windup_limit:
            self.integral = -self.windup_limit

        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output