import math
import time


class PIDcontroller:
    def __init__(self, kp, ki, kd, alpha, beta, max_theta, conversion="linear"):
        """
        PID Controller for 2D tracking (x, y → θ, φ)

        kp, ki, kd:
            Standard PID gains
            - kp: proportional gain (reaction strength)
            - ki: integral gain (accumulated error correction)
            - kd: derivative gain (predictive damping)

        alpha:
            Exponential smoothing factor (0 → no update, 1 → no smoothing)
            output = α * new + (1-α) * previous

        beta:
            Scaling factor that converts error magnitude → tilt angle

        max_theta:
            Maximum allowed tilt angle (safety limit)

        conversion:
            "linear" → θ = β * r
            "tanh"   → θ = tanh(β * r) (smooth saturation)
        """

        # Store PID gains
        self.kp, self.ki, self.kd = kp, ki, kd

        # Output smoothing factor
        self.alpha = alpha  

        # Scaling factor for magnitude → angle
        self.beta = beta  

        # Max allowed tilt
        self.max_theta = max_theta

        # Choose conversion method
        if conversion == "linear":
            self.magnitude_convert = 1   # Linear scaling
        elif conversion == "tanh":
            self.magnitude_convert = 0   # Nonlinear saturation
        else:
            self.magnitude_convert = -1  # Invalid (fallback)


        # --- Previous values (for derivative + filtering) ---
        self.prev_out_x = 0.0
        self.prev_out_y = 0.0

        self.prev_err_x = 0.0  
        self.prev_err_y = 0.0

        # --- Integral terms ---
        self.sum_err_x = 0.0
        self.sum_err_y = 0.0
        
        # Time tracking (for dt)
        self.last_time = None


    def pid(self, target, current):
        """
        Compute PID output

        target  = [x_target, y_target]
        current = [x_current, y_current]

        Returns:
            theta → tilt magnitude (degrees)
            phi   → direction (degrees)
        """

        # --------------------------------------------------
        # 1. Compute time difference (dt)
        # --------------------------------------------------
        new_time = time.perf_counter()

        # If first run, assume small dt to avoid division by zero
        dt = new_time - self.last_time if self.last_time is not None else 0.001


        # --------------------------------------------------
        # 2. Compute errors
        # --------------------------------------------------

        # NOTE:
        # error = current - target
        # (you may flip this depending on your coordinate system)
        err_x = current[0] - target[0]
        err_y = current[1] - target[1]

        # Integral (accumulate error over time)
        self.sum_err_x += err_x * dt
        self.sum_err_y += err_y * dt

        # Derivative (rate of change of error)
        d_err_x = (err_x - self.prev_err_x) / dt if dt > 0 else 0
        d_err_y = (err_y - self.prev_err_y) / dt if dt > 0 else 0


        # --------------------------------------------------
        # 3. PID output (x and y separately)
        # --------------------------------------------------

        pid_x = self.kp * err_x + self.ki * self.sum_err_x + self.kd * d_err_x
        pid_y = self.kp * err_y + self.ki * self.sum_err_y + self.kd * d_err_y


        # --------------------------------------------------
        # 4. Apply smoothing filter
        # --------------------------------------------------
        filtered_x = self.alpha * pid_x + (1 - self.alpha) * self.prev_out_x
        filtered_y = self.alpha * pid_y + (1 - self.alpha) * self.prev_out_y


        # --------------------------------------------------
        # 5. Convert (x, y) → polar (θ, φ)
        # --------------------------------------------------

        # Direction angle (0–360 degrees)
        phi = math.degrees(math.atan2(filtered_y, filtered_x))
        if phi < 0:
            phi += 360

        # Magnitude of correction
        r = math.sqrt(filtered_x**2 + filtered_y**2)


        # --------------------------------------------------
        # 6. Convert magnitude → tilt angle θ
        # --------------------------------------------------

        if self.magnitude_convert == 1:
            # Linear scaling
            theta = min(max(0, self.beta * r), self.max_theta)

        else:
            # Smooth nonlinear scaling (better for stability)
            theta = max(0, 15 * math.tanh(self.beta * r))


        # --------------------------------------------------
        # 7. Save values for next iteration
        # --------------------------------------------------
        self.prev_err_x = err_x
        self.prev_err_y = err_y

        self.prev_out_x = filtered_x
        self.prev_out_y = filtered_y

        self.last_time = new_time


        # --------------------------------------------------
        # 8. Return control output
        # --------------------------------------------------
        return theta, phi   # (degrees)