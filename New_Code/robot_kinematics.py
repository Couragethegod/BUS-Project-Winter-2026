import math


class RobotKinematics:
    def __init__(self, lp=7.0, l1=10.0, l2=9.6, lb=7.4, invert=False):
        self.lp = lp
        self.l1 = l1
        self.l2 = l2
        self.lb = lb
        self.invert = invert

        self.maxh = self.compute_maxh() - 0.2
        self.minh = self.compute_minh() + 0.45
        self.h = (self.maxh + self.minh) / 2.0
        self.maxtheta = 10.0

        self.A1 = [0.0, 0.0, 0.0]
        self.A2 = [0.0, 0.0, 0.0]
        self.A3 = [0.0, 0.0, 0.0]

        self.B1 = [0.0, 0.0, 0.0]
        self.B2 = [0.0, 0.0, 0.0]
        self.B3 = [0.0, 0.0, 0.0]

        self.C1 = [0.0, 0.0, 0.0]
        self.C2 = [0.0, 0.0, 0.0]
        self.C3 = [0.0, 0.0, 0.0]

        self.theta1 = 0.0
        self.theta2 = 0.0
        self.theta3 = 0.0
        self.max_theta(self.h)

    def compute_maxh(self):
        return math.sqrt(((self.l1 + self.l2) ** 2) - ((self.lp - self.lb) ** 2))

    def compute_minh(self):
        if self.l1 > self.l2:
            return math.sqrt((self.l1**2) - ((self.lb + self.l2 - self.lp) ** 2))
        if self.l2 > self.l1:
            return math.sqrt(((self.l2 - self.l1) ** 2) - ((self.lp - self.lb) ** 2))
        return 0.0

    def joint_angles_deg(self):
        return tuple(math.degrees(angle) for angle in (self.theta1, self.theta2, self.theta3))

    def solve_inverse_kinematics_spherical(self, theta, phi, h):
        self.h = h
        self.max_theta(h)
        theta = max(0.0, min(theta, self.maxtheta))

        a = math.sin(math.radians(theta)) * math.cos(math.radians(phi))
        b = math.sin(math.radians(theta)) * math.sin(math.radians(phi))
        c = math.cos(math.radians(theta))
        self.solve_inverse_kinematics_vector(a, b, c, h)

    def solve_inverse_kinematics_vector(self, a, b, c, h):
        self.B1 = [-0.5 * self.lb, math.sqrt(3) * 0.5 * self.lb, 0.0]
        self.B2 = [self.lb, 0.0, 0.0]
        self.B3 = [-0.5 * self.lb, -1.0 * math.sqrt(3) * 0.5 * self.lb, 0.0]

        self.solve_top(a, b, c, h)
        self.solve_middle()

        self.theta1 = math.pi / 2 - math.atan2(
            math.sqrt(self.C1[0] ** 2 + self.C1[1] ** 2) - self.lb,
            self.C1[2],
        )
        self.theta2 = math.atan2(self.C2[2], self.C2[0] - self.lb)
        self.theta3 = math.pi / 2 - math.atan2(
            math.sqrt(self.C3[0] ** 2 + self.C3[1] ** 2) - self.lb,
            self.C3[2],
        )

    def solve_top(self, a, b, c, h):
        denom_1 = math.sqrt(4 * c**2 + (a - math.sqrt(3) * b) ** 2)
        denom_2 = math.sqrt(c**2 + a**2)
        denom_3 = math.sqrt(4 * c**2 + (a + math.sqrt(3) * b) ** 2)

        self.A1 = [
            -(self.lp * c) / denom_1,
            (math.sqrt(3) * self.lp * c) / denom_1,
            h + ((a - math.sqrt(3) * b) * self.lp) / denom_1,
        ]
        self.A2 = [
            (self.lp * c) / denom_2,
            0.0,
            h - (self.lp * a) / denom_2,
        ]
        self.A3 = [
            -(self.lp * c) / denom_3,
            -(math.sqrt(3) * self.lp * c) / denom_3,
            h + ((a + math.sqrt(3) * b) * self.lp) / denom_3,
        ]

    def solve_middle(self):
        a11, a12, a13 = map(float, self.A1)
        a21, a22, a23 = map(float, self.A2)
        a31, a32, a33 = map(float, self.A3)

        p1 = (-a11 + math.sqrt(3) * a12 - 2 * self.lb) / a13
        q1 = (a11**2 + a12**2 + a13**2 + self.l2**2 - self.l1**2 - self.lb**2) / (2 * a13)
        r1 = p1**2 + 4
        s1 = 2 * p1 * q1 + 4 * self.lb
        t1 = q1**2 + self.lb**2 - self.l2**2

        p2 = (self.lb - a21) / a23
        q2 = (a21**2 + a23**2 - self.lb**2 + self.l2**2 - self.l1**2) / (2 * a23)
        r2 = p2**2 + 1
        s2 = 2 * (p2 * q2 - self.lb)
        t2 = q2**2 - self.l2**2 + self.lb**2

        p3 = (-a31 - math.sqrt(3) * a32 - 2 * self.lb) / a33
        q3 = (a31**2 + a32**2 + a33**2 + self.l2**2 - self.l1**2 - self.lb**2) / (2 * a33)
        r3 = p3**2 + 4
        s3 = 2 * p3 * q3 + 4 * self.lb
        t3 = q3**2 + self.lb**2 - self.l2**2

        c11 = (-s1 - math.sqrt(self._discriminant(r1, s1, t1))) / (2 * r1)
        c12 = -math.sqrt(3) * c11
        c13 = self._safe_sqrt(self.l2**2 - 4 * (c11**2) - 4 * self.lb * c11 - self.lb**2)

        c21 = (-s2 + math.sqrt(self._discriminant(r2, s2, t2))) / (2 * r2)
        c22 = 0.0
        c23 = self._safe_sqrt(self.l2**2 - (c21 - self.lb) ** 2)

        c31 = (-s3 - math.sqrt(self._discriminant(r3, s3, t3))) / (2 * r3)
        c32 = math.sqrt(3) * c31
        c33 = self._safe_sqrt(self.l2**2 - 4 * (c31**2) - 4 * self.lb * c31 - self.lb**2)

        z_sign = -1.0 if self.invert else 1.0
        self.C1 = [c11, c12, z_sign * c13]
        self.C2 = [c21, c22, z_sign * c23]
        self.C3 = [c31, c32, z_sign * c33]

    def max_theta(self, h, tol=1e-3):
        theta_low = 0.0
        theta_high = math.radians(20)

        def valid(theta):
            c = math.cos(theta)
            for sign in (1, -1):
                a21 = self.lp * c
                a23 = h - self.lp * (sign * math.sin(theta))
                try:
                    p2 = (self.lb - a21) / a23
                    q2 = (a21**2 + a23**2 - self.lb**2 + self.l2**2 - self.l1**2) / (2 * a23)
                    r2 = p2**2 + 1
                    s2 = 2 * (p2 * q2 - self.lb)
                    t2 = q2**2 - self.l2**2 + self.lb**2
                    disc = self._discriminant(r2, s2, t2)
                    c21 = (-s2 + math.sqrt(disc)) / (2 * r2)
                    delta = self._safe_sqrt(self.l2**2 - (c21 - self.lb) ** 2)
                    if abs(math.sqrt((a21 - c21) ** 2 + (a23 - delta) ** 2) - self.l1) > 1e-3:
                        return False
                    if abs(math.sqrt((self.lb - c21) ** 2 + delta**2) - self.l2) > 1e-3:
                        return False
                except ValueError:
                    return False
            return True

        while theta_high - theta_low > tol:
            theta_mid = (theta_low + theta_high) / 2
            if valid(theta_mid):
                theta_low = theta_mid
            else:
                theta_high = theta_mid

        self.maxtheta = max(0.0, math.degrees(round(theta_low, 4)) - 0.5)

    @staticmethod
    def _discriminant(r, s, t):
        disc = s**2 - 4 * r * t
        if disc < -1e-9:
            raise ValueError("No real inverse-kinematics solution for the requested pose.")
        return max(0.0, disc)

    @staticmethod
    def _safe_sqrt(value):
        if value < -1e-9:
            raise ValueError("Encountered an invalid geometry while solving inverse kinematics.")
        return math.sqrt(max(0.0, value))
