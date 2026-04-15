# Import your custom servo control module
import class_servo as cs

# Math library for trigonometry and square roots
import math

# Time library for delays
import time


class BBrobot:
    
    def __init__(self, ids):
        """
        Robot Initialization
        
        ids: list of 3 servo IDs
            Example: [1, 2, 3]
        """

        # Create ONE serial port (shared)
        self.port = cs.sPort()
    
        # Initialize 3 servo motors
        self.s1 = cs.RS304MD(ids[0])
        self.s2 = cs.RS304MD(ids[7])
        self.s3 = cs.RS304MD(ids[8])
        
        # Robot geometry (IMPORTANT)
        # These are link lengths (in meters)
        #
        # You MUST change these if your robot dimensions differ
        self.L = [
            0.04,   # Base arm length
            0.04,   # Upper arm length
            0.065,  # Forearm length
            0.065   # End-effector offset
        ]
       
        # Initial position (theta, phi, z)
        # theta = direction (degrees)
        # phi = tilt angle (degrees)
        # z = height (meters)
        self.ini_pos = [0, 0, 0.0632]

        # Safety limits (VERY IMPORTANT)
        self.pz_max = 0.0732   # Maximum height
        self.pz_min = 0.0532   # Minimum height

        # Maximum tilt angle (degrees)
        self.phi_max = 20


    def set_up(self):
        """
        Enable all servo motors (turn torque ON)
        """
        self.s1.trq_set(1)
        self.s2.trq_set(1)
        self.s3.trq_set(1)
        
    
    def clean_up(self):
        """
        Disable all motors and close serial connection
        """
        self.s1.trq_set(0)
        self.s2.trq_set(0)
        self.s3.trq_set(0)
        
        self.port.close_sPort()


    def kinema_inv(self, n, Pz):
        """
        Inverse Kinematics
        
        Converts desired platform orientation + height into servo angles.
        
        Parameters:
        n  : normal vector [x, y, z]
             (represents platform tilt direction)
        
        Pz : height of platform
        
        Returns:
        [theta_a, theta_b, theta_c] → angles for 3 motors
        """

        # Shortcut for link lengths
        L = self.L

        # ------------------- CENTRAL GEOMETRY -------------------
        # Solve for middle joint position using geometry equations
        
        A = (L[0] + L[1]) / Pz
        B = (Pz**2 + L[2]**2 - (L[0]+L[1])**2 - L[3]**2) / (2 * Pz)
        C = A**2 + 1
        D = 2 * (A * B - (L[0] + L[1]))
        E = B**2 + (L[0]+L[1])**2 - L[2]**2

        # Solve quadratic equation
        Pmx = (-D + math.sqrt(D**2 - 4*C*E)) / (2*C)

        # Z coordinate of midpoint
        Pmz = math.sqrt(
            L[2]**2 - Pmx**2 + 2*(L[0]+L[1])*Pmx - (L[0]+L[1])**2
        )

        # ------------------- ARM A -------------------
        # Compute joint positions for first motor
        
        a_m_x = (L[3] / math.sqrt(n[0]**2 + n[2]**2)) * (n[2])
        a_m_y = 0
        a_m_z = Pz + (L[3] / math.sqrt(n[0]**2 + n[2]**2)) * (-n[0])

        A_m = [a_m_x, a_m_y, a_m_z]

        # Solve triangle geometry
        A = (L[0] - A_m[0]) / A_m[2]
        B = (A_m[0]**2 + A_m[1]**2 + A_m[2]**2 - L[2]**2 - L[0]**2 + L[1]**2) / (2*A_m[2])
        C = A**2 + 1
        D = 2 * (A*B - L[0])
        E = B**2 + L[0]**2 - L[1]**2

        ax = (-D + math.sqrt(D**2 - 4*C*E)) / (2*C)
        ay = 0
        az = math.sqrt(L[1]**2 - ax**2 + 2*L[0]*ax - L[0]**2)

        # Adjust sign depending on geometry
        if (a_m_z < Pmz):
            az = -az

        # Final joint position
        A_2 = [ax, ay, az]

        # Convert to angle
        theta_a = 90 - math.degrees(math.atan2(A_2[0] - L[0], A_2[2]))

        # ------------------- ARM B -------------------
        # Similar calculations for second motor
        
        b_m_x = (L[3] / math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 + 2*math.sqrt(3)*n[0]*n[1])) * (-n[2])
        b_m_y = (L[3] / math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 + 2*math.sqrt(3)*n[0]*n[1])) * (-math.sqrt(3)*n[2])
        b_m_z = Pz + (L[3] / math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 + 2*math.sqrt(3)*n[0]*n[1])) * (math.sqrt(3)*n[1] + n[0])

        B_m = [b_m_x, b_m_y, b_m_z]

        A = -(B_m[0] + math.sqrt(3)*B_m[1] + 2*L[0]) / B_m[2]
        B = (B_m[0]**2 + B_m[1]**2 + B_m[2]**2 + L[1]**2 - L[2]**2 - L[0]**2) / (2*B_m[2])
        C = A**2 + 4
        D = 2*A*B + 4*L[0]
        E = B**2 + L[0]**2 - L[1]**2

        x = (-D - math.sqrt(D**2 - 4*C*E)) / (2*C)
        y = math.sqrt(3) * x
        z = math.sqrt(L[1]**2 - 4*x**2 - 4*L[0]*x - L[0]**2)

        if (b_m_z < Pmz):
            z = -z

        B_2 = [x, y, z]

        theta_b = 90 - math.degrees(
            math.atan2(math.sqrt(B_2[0]**2 + B_2[1]**2) - L[0], B_2[2])
        )

        # ------------------- ARM C -------------------
        # Third motor (mirror of B)
        
        c_m_x = (L[3] / math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 - 2*math.sqrt(3)*n[0]*n[1])) * (-n[2])
        c_m_y = (L[3] / math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 - 2*math.sqrt(3)*n[0]*n[1])) * (math.sqrt(3)*n[2])
        c_m_z = Pz + (L[3] / math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 - 2*math.sqrt(3)*n[0]*n[1])) * (-math.sqrt(3)*n[1] + n[0])

        C_m = [c_m_x, c_m_y, c_m_z]

        A = -(C_m[0] - math.sqrt(3)*C_m[1] + 2*L[0]) / C_m[2]
        B = (C_m[0]**2 + C_m[1]**2 + C_m[2]**2 + L[1]**2 - L[2]**2 - L[0]**2) / (2*C_m[2])
        C = A**2 + 4
        D = 2*A*B + 4*L[0]
        E = B**2 + L[0]**2 - L[1]**2

        x = (-D - math.sqrt(D**2 - 4*C*E)) / (2*C)
        y = -math.sqrt(3) * x
        z = math.sqrt(L[1]**2 - 4*x**2 - 4*L[0]*x - L[0]**2)

        if (c_m_z < Pmz):
            z = -z

        C_2 = [x, y, z]

        theta_c = 90 - math.degrees(
            math.atan2(math.sqrt(C_2[0]**2 + C_2[1]**2) - L[0], C_2[2])
        )

        # Return all three motor angles
        return [theta_a, theta_b, theta_c]

    
    def control_t_posture(self, pos, t):
        """
        Move robot to a desired posture
        
        pos = [theta, phi, z]
            theta → direction (degrees)
            phi   → tilt angle (degrees)
            z     → height (meters)
        
        t = time (seconds)
        """

        theta = pos[0]
        phi = pos[1]

        # Limit tilt angle (safety)
        if phi > self.phi_max:
            phi = self.phi_max

        Pz = pos[2]

        # Clamp height within safe range
        if Pz > self.pz_max:
            Pz = self.pz_max
        elif Pz < self.pz_min:
            Pz = self.pz_min 

        # Convert spherical angles to normal vector
        z = math.cos(math.radians(phi))
        r = math.sin(math.radians(phi))
        x = r * math.cos(math.radians(theta))
        y = r * math.sin(math.radians(theta))

        n = [x, y, z]

        # Compute motor angles using inverse kinematics
        angles = self.kinema_inv(n, Pz)

        # Send commands to servos (move over time t)
        self.s1.control_time_rotate(angles[0], t)
        self.s2.control_time_rotate(angles[1], t)
        self.s3.control_time_rotate(angles[2], t)

        # Wait for motion to complete
        time.sleep(t)
    
    def Initialize_posture(self):
        """
        Move robot to initial safe position
        """
        pos = self.ini_pos
        t = 1  # move over 1 second
        self.control_t_posture(pos, t)