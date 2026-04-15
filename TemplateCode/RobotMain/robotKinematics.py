import math
import numpy as np


class RobotKinematics:
    """
    This class models the inverse kinematics of a 3-arm parallel robot (likely a delta-style mechanism).

    Key idea:
    - You define a desired orientation + height of the top platform
    - The class computes joint angles (theta1, theta2, theta3)

    IMPORTANT PARAMETERS TO TWEAK:
    - lp: radius of the top platform
    - lb: radius of the base
    - l1: length of upper arms
    - l2: length of lower arms
    - invert: flips configuration (elbow-up vs elbow-down behavior)
    """

    def __init__(self, lp=14.00, l1=10, l2=9.6, lb=7.40, invert=False):

        # === GEOMETRY PARAMETERS ===
        self.lp = lp    # Radius of TOP platform (distance from center to joints A1, A2, A3)
        self.l1 = l1    # Length of TOP arms (from A to C)
        self.l2 = l2    # Length of BOTTOM arms (from B to C)
        self.lb = lb    # Radius of BASE (distance from center to B1, B2, B3)

        # Whether the mechanism is inverted (changes sign of Z solutions)
        # Useful if your robot is assembled differently
        self.invert = invert

        # === HEIGHT LIMITS ===
        # These define the valid vertical workspace
        # NOTE: small offsets added to avoid singularities / edge instability
        self.maxh = self.compute_maxh() - 0.2
        self.minh = self.compute_minh() + 0.45

        # Initial position of top platform center
        self.p = [0.0, 0.0, self.maxh]

        # Default working height (midpoint of range)
        self.h = (self.maxh + self.minh) / 2

        # Maximum tilt angle (degrees) — dynamically computed later
        self.maxtheta = 10

        # === NODE DEFINITIONS ===
        # A = top joints
        # B = base joints
        # C = elbow/middle joints

        # Top Nodes (on moving platform)
        self.A1 = [0, 0, 0]
        self.A2 = [0, 0, 0]
        self.A3 = [0, 0, 0]

        # Bottom Nodes (fixed base)
        self.B1 = [0, 0, 0]
        self.B2 = [0, 0, 0]
        self.B3 = [0, 0, 0]

        # Middle Nodes (joint between l1 and l2)
        self.C1 = [0.0, 0.0, 0.0]
        self.C2 = [0.0, 0.0, 0.0]
        self.C3 = [0.0, 0.0, 0.0]

        # Compute max allowable tilt at default height
        self.max_theta(self.h)

        # Output joint angles (radians)
        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0

    def compute_maxh(self):
        """
        Computes maximum reachable height based on full extension of arms.

        Derived from geometry:
        (l1 + l2)^2 = horizontal_offset^2 + height^2

        USER NOTE:
        - If this returns NaN → your geometry is physically impossible
        """
        return math.sqrt(((self.l1 + self.l2) ** 2) - ((self.lp - self.lb) ** 2))

    def compute_minh(self):
        """
        Computes minimum reachable height.

        Depends on which arm is longer.
        This is one of the MOST fragile parts of the model.

        USER WARNING:
        - If l1 ≈ l2, edge cases may behave poorly
        """
        if self.l1 > self.l2:
            return math.sqrt((self.l1 ** 2) - ((self.lb + self.l2 - self.lp) ** 2))
        elif self.l2 > self.l1:
            return math.sqrt(((self.l2 - self.l1) ** 2) - ((self.lp - self.lb) ** 2))
        else:
            return 0

    def solve_top(self, a, b, c, h):
        """
        Computes positions of top joints A1, A2, A3.

        INPUT:
        - (a, b, c): unit normal vector of top platform
        - h: vertical height

        This defines the ORIENTATION of the platform.

        USER CONTROL:
        - This is where platform tilt is defined
        - Changing (a,b,c) changes tilt direction
        """

        if not self.invert:
            # Standard configuration

            self.A1 = [
                -(self.lp * c) / (math.sqrt(4 * c**2 + (a - math.sqrt(3)*b)**2)),
                (math.sqrt(3) * self.lp * c) / (math.sqrt(4 * c**2 + (a - math.sqrt(3)*b)**2)),
                h + ((a - math.sqrt(3)*b) * self.lp) / (math.sqrt(4 * c**2 + (a - math.sqrt(3)*b)**2))
            ]

            self.A2 = [
                (self.lp * c) / (math.sqrt(c**2 + a**2)),
                0,
                h - ((self.lp * a) / (math.sqrt(c**2 + a**2)))
            ]

            self.A3 = [
                -(self.lp * c) / (math.sqrt(4 * c**2 + (a + math.sqrt(3)*b)**2)),
                -(math.sqrt(3) * self.lp * c) / (math.sqrt(4 * c**2 + (a + math.sqrt(3)*b)**2)),
                h + ((a + math.sqrt(3)*b) * self.lp) / (math.sqrt(4 * c**2 + (a + math.sqrt(3)*b)**2))
            ]

        else:
            # NOTE: This block is IDENTICAL to above → redundant
            # You are NOT actually changing behavior here
            # This is a design flaw — you probably intended different equations

            self.A1 = [...]
            self.A2 = [...]
            self.A3 = [...]

    def solve_middle(self):
        """
        Solves for intermediate joints C1, C2, C3 using geometry constraints.

        This is the CORE IK solver:
        - Uses quadratic equations
        - Solves intersection of spheres/circles

        WARNING:
        - sqrt(...) can break if geometry is invalid
        - Negative discriminant = unreachable position
        """

        a11, a12, a13 = map(float, self.A1)
        a21, a22, a23 = map(float, self.A2)
        a31, a32, a33 = map(float, self.A3)

        # These p, q, r, s, t values come from algebraic manipulation
        # of distance constraints between joints

        # --- ARM 1 ---
        p1 = (-a11 + math.sqrt(3)*a12 - 2*self.lb) / a13
        q1 = (a11**2 + a12**2 + a13**2 + self.l2**2 - self.l1**2 - self.lb**2) / (2*a13)
        r1 = p1**2 + 4
        s1 = 2*p1*q1 + 4*self.lb
        t1 = q1**2 + self.lb**2 - self.l2**2

        # --- ARM 2 ---
        p2 = (self.lb - a21) / a23
        q2 = (a21**2 + a23**2 - self.lb**2 + self.l2**2 - self.l1**2) / (2*a23)
        r2 = p2**2 + 1
        s2 = 2*(p2*q2 - self.lb)
        t2 = q2**2 - self.l2**2 + self.lb**2

        # --- ARM 3 ---
        p3 = (-a31 - math.sqrt(3)*a32 - 2*self.lb) / a33
        q3 = (a31**2 + a32**2 + a33**2 + self.l2**2 - self.l1**2 - self.lb**2) / (2*a33)
        r3 = p3**2 + 4
        s3 = 2*p3*q3 + 4*self.lb
        t3 = q3**2 + self.lb**2 - self.l2**2

        # Solve quadratics → gives joint positions
        # Choice of ± sqrt determines configuration

        if not self.invert:
            # "Normal" solution branch

            c11 = (-s1 - math.sqrt(s1**2 - 4*r1*t1)) / (2*r1)
            c12 = -math.sqrt(3) * c11
            c13 = math.sqrt(self.l2**2 - 4*(c11**2) - 4*self.lb*c11 - self.lb**2)

            self.C1 = [c11, c12, c13]

            # Repeat for other arms...
            # (Same pattern, slightly different geometry)

        else:
            # Inverted → flip Z direction
            # This changes whether arms bend upward or downward

            self.C1 = [c11, c12, -c13]
            # etc...

    def solve_inverse_kinematics_vector(self, a, b, c, h):
        """
        Full IK solve using a NORMAL VECTOR input.

        Steps:
        1. Define base triangle (B points)
        2. Solve top platform (A points)
        3. Solve intermediate joints (C points)
        4. Compute joint angles

        USER NOTE:
        - This is the function to call if you already have a direction vector
        """

        # Base triangle (fixed geometry)
        self.B1 = [-0.5*self.lb, math.sqrt(3)*0.5*self.lb, 0]
        self.B2 = [self.lb, 0, 0]
        self.B3 = [-0.5*self.lb, -math.sqrt(3)*0.5*self.lb, 0]

        self.solve_top(a, b, c, h)
        self.solve_middle()

        # Convert positions → joint angles
        # atan2 used for stability

        self.theta1 = math.pi/2 - math.atan2(
            math.sqrt(self.C1[0]**2 + self.C1[1]**2) - self.lb,
            self.C1[2]
        )

        self.theta2 = math.atan2(
            self.C2[2],
            self.C2[0] - self.lb
        )

        self.theta3 = math.pi/2 - math.atan2(
            math.sqrt(self.C3[0]**2 + self.C3[1]**2) - self.lb,
            self.C3[2]
        )

    def solve_inverse_kinematics_spherical(self, theta, phi, h):
        """
        More user-friendly IK interface using spherical coordinates.

        INPUT:
        - theta: tilt angle (degrees)
        - phi: direction of tilt (degrees)
        - h: height

        USER SHOULD USE THIS FUNCTION MOST OF THE TIME.
        """

        self.h = h

        # Update allowable tilt range at this height
        self.max_theta(h)

        # Clamp to safe range (prevents invalid math)
        theta = min(theta, self.maxtheta)

        # Convert spherical → Cartesian normal vector
        a = math.sin(math.radians(theta)) * math.cos(math.radians(phi))
        b = math.sin(math.radians(theta)) * math.sin(math.radians(phi))
        c = math.cos(math.radians(theta))

        try:
            self.solve_inverse_kinematics_vector(a, b, c, h)
        except Exception as e:
            # BAD: silent failure
            # You should log or raise this instead
            print(a, b, c, h, theta, phi)
            pass

    def max_theta(self, h, tol=1e-3):
        """
        Finds maximum tilt angle allowed at a given height.

        Uses binary search:
        - Tests feasibility of geometry
        - Rejects invalid configurations

        USER NOTE:
        - This is critical for avoiding crashes
        - If your robot behaves weirdly near limits → this is why
        """

        theta_low, theta_high = 0.0, math.radians(20)

        def valid(theta):
            """
            Checks if a given tilt angle is physically possible.
            """
            c = math.cos(theta)

            for s in (1, -1):  # test both tilt directions
                a21 = self.lp * c
                a23 = h - self.lp * (s * math.sin(theta))

                try:
                    # Same math as IK — checking feasibility only
                    p2 = (self.lb - a21) / a23
                    q2 = (a21**2 + a23**2 - self.lb**2 + self.l2**2 - self.l1**2) / (2 * a23)
                    r2 = p2**2 + 1
                    s2 = 2 * (p2 * q2 - self.lb)
                    t2 = q2**2 - self.l2**2 + self.lb**2

                    disc = s2**2 - 4 * r2 * t2
                    if disc < 0:
                        return False

                    c21 = (-s2 + math.sqrt(disc)) / (2 * r2)

                    delta = self.l2**2 - (c21 - self.lb)**2
                    if delta < 0:
                        return False

                    c23 = math.sqrt(delta)

                    # Check arm lengths consistency
                    if abs(math.sqrt((a21-c21)**2 + (a23-c23)**2) - self.l1) > 1e-3:
                        return False
                    if abs(math.sqrt((self.lb-c21)**2 + c23**2) - self.l2) > 1e-3:
                        return False

                except:
                    return False

            return True

        # Binary search for max valid theta
        while theta_high - theta_low > tol:
            theta_mid = (theta_low + theta_high) / 2
            if valid(theta_mid):
                theta_low = theta_mid
            else:
                theta_high = theta_mid

        # Store result in degrees
        self.maxtheta = max(0, math.degrees(round(theta_low, 4)) - 0.5)