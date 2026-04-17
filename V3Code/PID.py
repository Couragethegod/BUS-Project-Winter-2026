import time

class PIDcontroller:
    def __init__(
        self,
        kp,
        ki,
        kd,
        integral_limit=10.0,
        output_limit=None,
        derivative_alpha=0.2,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit) if integral_limit is not None else None
        self.output_limit = output_limit
        self.derivative_alpha = max(0.0, min(1.0, derivative_alpha))
        self.reset()

    def reset(self):
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.integral = 0.0
        self.last_time = None

    def update(self, error):
        now = time.perf_counter()
        dt = 0.02 if self.last_time is None else min(0.05, max(1e-3, now - self.last_time))
        self.last_time = now
        self.integral += error * dt

        if self.integral_limit is not None:
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

        raw_derivative = (error - self.prev_error) / dt
        derivative = (
            self.derivative_alpha * raw_derivative
            + (1.0 - self.derivative_alpha) * self.prev_derivative
        )
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        if self.output_limit is not None:
            output = max(-self.output_limit, min(self.output_limit, output))

        self.prev_error = error
        self.prev_derivative = derivative
        return output
