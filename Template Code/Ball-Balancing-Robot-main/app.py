from flask import Flask, render_template, request, jsonify
from flask_cors import CORS
import math

# Your custom kinematics + controller modules
from robotKinematics import RobotKinematics
from controller import RobotController
a
# Create Flask app
app = Flask(__name__)

# Enable CORS so frontend (likely JS) can talk to backend
# IMPORTANT: In production, restrict origins instead of allowing all
CORS(app)

# === ROBOT INITIALIZATION ===

# Core kinematics model
robot = RobotKinematics()

# Controller that likely sends commands to motors (hardware layer)
rc = RobotController(robot)

# === HARD-CODED GEOMETRY PARAMETERS ===
# These define the physical robot dimensions

robot.lp = 7.125   # Top platform radius
robot.l1 = 6.2     # Upper arm length
robot.l2 = 4.5     # Lower arm length
robot.lb = 4.0     # Base radius

# ⚠️ DESIGN ISSUE:
# You're setting these AFTER initializing RobotKinematics
# → any values computed in __init__ may now be outdated

# === WORKSPACE LIMIT FUNCTIONS ===

def compute_maxh(l1, l2, lp, lb):
    """
    Maximum reachable height based on full arm extension.

    If this becomes complex (sqrt negative), geometry is invalid.
    """
    return math.sqrt(((l1 + l2) ** 2) - ((lp - lb) ** 2))


def compute_minh(l1, l2, lp, lb):
    """
    Minimum reachable height.

    Depends on which arm is longer.
    """
    if l1 > l2:
        return math.sqrt((l1 ** 2) - ((lb + l2 - lp) ** 2))
    elif l2 > l1:
        return math.sqrt(((l2 - l1) ** 2) - ((lp - lb) ** 2))
    else:
        return 0

# Apply workspace limits with safety margins
# These margins prevent singularities / instability

robot.maxh = compute_maxh(robot.l1, robot.l2, robot.lp, robot.lb) - 0.2
robot.minh = compute_minh(robot.l1, robot.l2, robot.lp, robot.lb) + 0.45

# ⚠️ These magic numbers (-0.2, +0.45) should be constants with names


# === ROUTES ===

@app.route("/")
def index():
    """
    Serves the main frontend page (likely contains sliders + visualization)
    """
    return render_template("index.html")


@app.route("/update", methods=["POST"])
def update_robot():
    """
    Main control endpoint.

    Receives slider values from frontend, converts them into:
    - orientation (theta, phi)
    - height (h)

    Then:
    - runs inverse kinematics
    - sends angles to controller
    - returns geometry for visualization
    """

    data = request.get_json()

    # === INPUT FROM FRONTEND ===
    # These are scaled integers from sliders

    slider_theta = float(data.get("slider_theta", 0))   # 0..2000 → 0..20 degrees
    slider_phi   = float(data.get("slider_phi", 0))     # 0..36000 → 0..360 degrees
    slider_h     = float(data.get("slider_h", 814))     # scaled by 100

    # Convert to real-world values
    theta_deg = slider_theta / 100.0
    phi_deg   = slider_phi   / 100.0
    h         = slider_h     / 100.0

    # Default orientation (flat platform)
    alpha = 0.0
    beta  = 0.0
    gamma = 1.0

    # Default fallback max tilt
    max_theta_for_h = 10.0

    try:
        # === CONVERT TO NORMAL VECTOR ===
        # This defines platform orientation

        theta_rad = math.radians(theta_deg)
        phi_rad   = math.radians(phi_deg)

        alpha = math.sin(theta_rad) * math.cos(phi_rad)
        beta  = math.sin(theta_rad) * math.sin(phi_rad)
        gamma = math.cos(theta_rad)

        # === COMPUTE MAX SAFE TILT ===
        # ⚠️ PROBLEM: your function doesn't return anything!
        max_theta_for_h = robot.max_theta(h)

        # === SOLVE INVERSE KINEMATICS ===
        robot.solve_inverse_kinematics_vector(alpha, beta, gamma, h)

        # === SEND TO MOTORS ===
        # Converts internal angles → motor angles

        rc.set_motor_angles(
            math.degrees(math.pi*0.5 - robot.theta1),
            math.degrees(math.pi*0.5 - robot.theta2),
            math.degrees(math.pi*0.5 - robot.theta3)
        )

    except:
        # ⚠️ CRITICAL PROBLEM:
        # This hides ALL errors (math errors, invalid geometry, controller issues)
        # → debugging becomes impossible
        pass

    # === RESPONSE FOR FRONTEND VISUALIZATION ===

    response = {
        # Orientation vector
        "alpha": alpha,
        "beta": beta,
        "gamma": gamma,

        # Height
        "h": h,

        # Joint angles (radians)
        "theta1": getattr(robot, "theta1", 0.0),
        "theta2": getattr(robot, "theta2", 0.0),
        "theta3": getattr(robot, "theta3", 0.0),

        # Geometry points (used for 3D rendering)
        "A_points": [robot.A1, robot.A2, robot.A3],
        "B_points": [robot.B1, robot.B2, robot.B3],
        "C_points": [robot.C1, robot.C2, robot.C3],

        # Lines for visualization
        "line_A": [
            robot.A1, robot.A2,
            robot.A2, robot.A3,
            robot.A3, robot.A1
        ],

        "line_B": [
            robot.B1, robot.B2,
            robot.B2, robot.B3,
            robot.B3, robot.B1
        ],

        # Arms (A → C → B)
        "line_C1": [robot.A1, robot.C1, robot.C1, robot.B1],
        "line_C2": [robot.A2, robot.C2, robot.C2, robot.B2],
        "line_C3": [robot.A3, robot.C3, robot.C3, robot.B3],

        # Geometry parameters (useful for frontend scaling/debug)
        "lp":   robot.lp,
        "l1":   robot.l1,
        "l2":   robot.l2,
        "lb":   robot.lb,

        # Workspace limits
        "minh": robot.minh,
        "maxh": robot.maxh,

        "max_theta": max_theta_for_h
    }

    return jsonify(response)


if __name__ == "__main__":
    """
    Runs the Flask server.

    host="0.0.0.0" → allows access from other devices on network
    debug=True → auto-reload + detailed errors (DO NOT use in production)
    """
    app.run(host="0.0.0.0", port=8000, debug=True)