import time


class PIDcontroller:
    def __init__(
        self,
        kp,
        ki,
        kd,
        windup_limit=10.0,
        output_limit=None,
        derivative_smoothing=0.25,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.windup_limit = abs(windup_limit) if windup_limit is not None else None
        self.output_limit = output_limit
        self.derivative_smoothing = min(max(derivative_smoothing, 0.0), 1.0)
        self.reset()

    def reset(self):
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.integral = 0.0
        self.last_time = None

    def update(self, error, dt=None):
        now = time.perf_counter()
        if dt is None:
            if self.last_time is None:
                dt = 0.02
            else:
                dt = max(1e-3, now - self.last_time)
        else:
            dt = max(1e-3, dt)

        self.last_time = now
        self.integral += error * dt

        if self.windup_limit is not None:
            self.integral = max(-self.windup_limit, min(self.windup_limit, self.integral))

        raw_derivative = (error - self.prev_error) / dt
        derivative = (
            self.derivative_smoothing * raw_derivative
            + (1.0 - self.derivative_smoothing) * self.prev_derivative
        )

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        if self.output_limit is not None:
            output = max(-self.output_limit, min(self.output_limit, output))

        self.prev_error = error
        self.prev_derivative = derivative
        return output
