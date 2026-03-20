# Import required libraries
import math   # For trigonometric functions and square root
import time   # For precise time measurements


class PID:
    def __init__(self, K_PID, k, alpha):
        """
        PID Controller Initialization
        
        Parameters:
        K_PID : list or tuple [Kp, Ki, Kd]
            Kp = Proportional gain (reacts to current error)
            Ki = Integral gain (reacts to accumulated past error)
            Kd = Derivative gain (reacts to rate of change of error)
        
        k : float
            Scaling factor for final output magnitude (phi)
            Think of this as "overall strength" of movement
        
        alpha : float (0 to 1)
            Smoothing factor for output (low-pass filter)
            - Closer to 1 → more responsive, less smooth
            - Closer to 0 → smoother, but slower response
        """

        # PID gains
        self.kp = K_PID[0]   # Proportional gain
        self.ki = K_PID[1]   # Integral gain
        self.kd = K_PID[2]   # Derivative gain

        # Output scaling factor
        self.k = k

        # Smoothing factor
        self.alpha = alpha

        # Store previous outputs (used for smoothing)
        self.last_output_x = 0
        self.last_output_y = 0

        # Store previous errors (used for derivative calculation)
        self.last_error_x = 0
        self.last_error_y = 0

        # Store accumulated error (integral term)
        self.integral_x = 0
        self.integral_y = 0

        # Store previous time (needed to compute time difference)
        self.last_time = None

        # These variables are currently unused but may be used later
        self.count = 0
        self.F = 0

    def compute(self, Goal, Current_value):
        """
        Compute PID output
        
        Parameters:
        Goal : tuple (x_target, y_target)
            Desired position
        
        Current_value : tuple (x_current, y_current)
            Current position
        
        Returns:
        theta : float
            Direction (angle in degrees, 0–360)
        
        phi : float
            Magnitude (how strong to move in that direction)
        """

        # Get current time with high precision
        current_time = time.perf_counter()

        # If this is the first run, initialize time and return zero output
        if self.last_time is None:
            self.last_time = current_time
            return 0, 0
        
        # Calculate error (difference between goal and current position)
        error_x = Goal[0] - Current_value[0]
        error_y = Goal[1] - Current_value[1]
        
        # Time difference (IMPORTANT for integral and derivative)
        dt = current_time - self.last_time

        # ------------------- INTEGRAL TERM -------------------
        # Accumulates error over time (helps eliminate steady-state error)
        self.integral_x += error_x * dt
        self.integral_y += error_y * dt
        
        # ------------------- DERIVATIVE TERM -------------------
        # Measures how fast error is changing (helps reduce overshoot)
        derivative_x = (error_x - self.last_error_x) / dt
        derivative_y = (error_y - self.last_error_y) / dt
        
        # ------------------- PID OUTPUT -------------------
        # Combine all three components
        output_x = (
            self.kp * error_x +
            self.ki * self.integral_x +
            self.kd * derivative_x
        )

        output_y = (
            self.kp * error_y +
            self.ki * self.integral_y +
            self.kd * derivative_y
        )
        
        # ------------------- OUTPUT SMOOTHING -------------------
        # Apply exponential smoothing (low-pass filter)
        # Reduces jitter/noise in output
        output_x = self.alpha * output_x + (1 - self.alpha) * self.last_output_x
        output_y = self.alpha * output_y + (1 - self.alpha) * self.last_output_y
        
        # ------------------- CONVERT TO POLAR -------------------
        # Convert (x, y) output into:
        # - theta (direction)
        # - phi (magnitude)

        # Angle in degrees (-180 to 180)
        theta = math.degrees(math.atan2(output_y, output_x))

        # Convert to 0–360 range
        if theta < 0:
            theta += 360

        # Magnitude (vector length), scaled by k
        phi = self.k * math.sqrt(output_x**2 + output_y**2)

        # ------------------- SAVE STATE -------------------
        # Store values for next iteration
        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_output_x = output_x
        self.last_output_y = output_y
        self.last_time = current_time

        # Return direction and magnitude
        return theta, phi