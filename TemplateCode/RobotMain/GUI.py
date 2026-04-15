# Standard libraries
import sys
import numpy as np
import math

# PyQt5 for GUI
from PyQt5 import QtCore, QtWidgets

# PyQtGraph for 3D visualization
import pyqtgraph.opengl as gl

# Your robot kinematics model
from robotKinematics import RobotKinematics  


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, robot):
        """
        Main GUI Window
        
        robot: instance of RobotKinematics (handles all math)
        """
        super().__init__()

        self.robot = robot  # Store robot model

        # Window setup
        self.setWindowTitle("3RRS Robot Simulator")
        self.resize(1000, 600)

        # Create main container and layout
        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QtWidgets.QHBoxLayout(main_widget)

        # --- Initial values for vector + height ---
        self.alpha = 0.0
        self.beta = 0.0
        self.gamma = 1.0
        self.h = 0.0 


        # =========================================================
        # LEFT SIDE → 3D VISUALIZATION
        # =========================================================

        # Create OpenGL 3D viewer
        self.gl_widget = gl.GLViewWidget()

        # Camera distance (zoom level)
        # 🔧 Increase to zoom out, decrease to zoom in
        self.gl_widget.opts['distance'] = 20  

        # Add to layout (takes 75% space)
        main_layout.addWidget(self.gl_widget, 3)

        # Add grid (reference plane)
        grid = gl.GLGridItem()
        grid.scale(2, 2, 1)  # scale grid size
        self.gl_widget.addItem(grid)

        # --- POINTS (A, B, C) ---
        # Each is a triangle of 3 points

        self.scatter_A = gl.GLScatterPlotItem(
            color=np.array([(1.0, 0.2, 0.2, 1.0)] * 3), size=10
        )
        self.scatter_B = gl.GLScatterPlotItem(
            color=np.array([(0.0, 0.57, 0.8, 1.0)] * 3), size=10
        )
        self.scatter_C = gl.GLScatterPlotItem(
            color=np.array([(0.99, 0.84, 0.57, 1.0)] * 3), size=10
        )

        self.gl_widget.addItem(self.scatter_A)
        self.gl_widget.addItem(self.scatter_B)
        self.gl_widget.addItem(self.scatter_C)

        # --- LINES connecting points ---
        self.line_A = gl.GLLinePlotItem(color=(1, 0.2, 0.2, 1), width=5)
        self.line_B = gl.GLLinePlotItem(color=(0, 0.57, 0.8, 1), width=5)

        self.gl_widget.addItem(self.line_A)
        self.gl_widget.addItem(self.line_B)

        # Lines from A → C → B (3 legs of robot)
        self.line_C1 = gl.GLLinePlotItem(color=(1, 0.84, 0.57, 1), width=5)
        self.line_C2 = gl.GLLinePlotItem(color=(1, 0.84, 0.57, 1), width=5)
        self.line_C3 = gl.GLLinePlotItem(color=(1, 0.84, 0.57, 1), width=5)

        self.gl_widget.addItem(self.line_C1)
        self.gl_widget.addItem(self.line_C2)
        self.gl_widget.addItem(self.line_C3)


        # =========================================================
        # RIGHT SIDE → INPUT PANEL
        # =========================================================

        self.input_panel = QtWidgets.QWidget()
        input_layout = QtWidgets.QVBoxLayout(self.input_panel)


        # -------------------------
        # ROBOT PARAMETER PANEL
        # -------------------------

        group_box = QtWidgets.QGroupBox("Robot Parameters")

        grid = QtWidgets.QGridLayout()

        # 🔧 These define robot geometry → VERY IMPORTANT
        # lp = platform radius
        # l1, l2 = link lengths
        # lb = base radius

        # lp
        grid.addWidget(QtWidgets.QLabel("lp:"), 0, 0)
        self.input_lp = QtWidgets.QDoubleSpinBox()
        self.input_lp.setValue(self.robot.lp)
        grid.addWidget(self.input_lp, 0, 1)

        # l1
        grid.addWidget(QtWidgets.QLabel("l1:"), 1, 0)
        self.input_l1 = QtWidgets.QDoubleSpinBox()
        self.input_l1.setValue(self.robot.l1)
        grid.addWidget(self.input_l1, 1, 1)

        # l2
        grid.addWidget(QtWidgets.QLabel("l2:"), 2, 0)
        self.input_l2 = QtWidgets.QDoubleSpinBox()
        self.input_l2.setValue(self.robot.l2)
        grid.addWidget(self.input_l2, 2, 1)

        # lb
        grid.addWidget(QtWidgets.QLabel("lb:"), 3, 0)
        self.input_lb = QtWidgets.QDoubleSpinBox()
        self.input_lb.setValue(self.robot.lb)
        grid.addWidget(self.input_lb, 3, 1)

        # Computed limits (read-only)
        grid.addWidget(QtWidgets.QLabel("maxh:"), 4, 0)
        self.label_maxh = QtWidgets.QLabel("")
        grid.addWidget(self.label_maxh, 4, 1)

        grid.addWidget(QtWidgets.QLabel("minh:"), 5, 0)
        self.label_minh = QtWidgets.QLabel("")
        grid.addWidget(self.label_minh, 5, 1)

        # Inversion toggle (flip robot orientation)
        grid.addWidget(QtWidgets.QLabel("Invert:"), 6, 0)
        self.input_invert = QtWidgets.QCheckBox()
        self.input_invert.setChecked(self.robot.invert)
        grid.addWidget(self.input_invert, 6, 1)

        # Apply button + auto update
        submit_button = QtWidgets.QPushButton("Apply")
        self.auto_update_checkbox = QtWidgets.QCheckBox("Auto Update")

        hbox = QtWidgets.QHBoxLayout()
        hbox.addWidget(submit_button)
        hbox.addWidget(self.auto_update_checkbox)

        grid.addLayout(hbox, 7, 0, 1, 2)

        group_box.setLayout(grid)
        input_layout.addWidget(group_box)


        # =========================================================
        # INPUT CONTROL (θ, φ, h)
        # =========================================================

        input_group = QtWidgets.QGroupBox("Input")
        grid2 = QtWidgets.QGridLayout()

        # θ (tilt angle)
        self.slider_theta = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_theta.setMaximum(1500)  # 15.00°
        self.spin_theta = QtWidgets.QDoubleSpinBox()
        self.spin_theta.setRange(0, 15)

        # φ (rotation angle)
        self.slider_phi = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_phi.setMaximum(36000)
        self.spin_phi = QtWidgets.QDoubleSpinBox()
        self.spin_phi.setRange(0, 360)

        # h (height)
        self.slider_h = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_h.setMaximum(10000)
        self.spin_h = QtWidgets.QDoubleSpinBox()

        # Add to layout
        grid2.addWidget(QtWidgets.QLabel("θ:"), 0, 0)
        grid2.addWidget(self.slider_theta, 0, 1)
        grid2.addWidget(self.spin_theta, 0, 2)

        grid2.addWidget(QtWidgets.QLabel("φ:"), 1, 0)
        grid2.addWidget(self.slider_phi, 1, 1)
        grid2.addWidget(self.spin_phi, 1, 2)

        grid2.addWidget(QtWidgets.QLabel("h:"), 2, 0)
        grid2.addWidget(self.slider_h, 2, 1)
        grid2.addWidget(self.spin_h, 2, 2)

        input_group.setLayout(grid2)
        input_layout.addWidget(input_group)


        # =========================================================
        # REAL-TIME UPDATE TIMER
        # =========================================================

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)

        # 🔧 Lower value → smoother animation but higher CPU usage
        self.timer.start(50)


        # =========================================================
        # SIGNAL CONNECTIONS
        # =========================================================

        # Sync sliders ↔ spinboxes
        self.slider_theta.valueChanged.connect(lambda v: self.spin_theta.setValue(v/100))
        self.spin_theta.valueChanged.connect(lambda v: self.slider_theta.setValue(int(v*100)))

        self.slider_phi.valueChanged.connect(lambda v: self.spin_phi.setValue(v/100))
        self.spin_phi.valueChanged.connect(lambda v: self.slider_phi.setValue(int(v*100)))

        self.slider_h.valueChanged.connect(lambda v: self.spin_h.setValue(v/100))
        self.spin_h.valueChanged.connect(lambda v: self.slider_h.setValue(int(v*100)))

        # Update robot when inputs change
        self.slider_theta.valueChanged.connect(self.update_inverse_kinematics)
        self.slider_phi.valueChanged.connect(self.update_inverse_kinematics)
        self.slider_h.valueChanged.connect(self.update_inverse_kinematics)

        submit_button.clicked.connect(self.validate_parameters)

        # Initialize
        self.validate_parameters()


    # =========================================================
    # CORE COMPUTATION FUNCTIONS
    # =========================================================

    def compute_vector(self):
        """
        Convert (θ, φ) → (α, β, γ) direction vector
        
        This defines platform orientation
        """

        theta = math.radians(self.slider_theta.value() / 100)
        phi   = math.radians(self.slider_phi.value() / 100)

        self.alpha = math.sin(theta) * math.cos(phi)
        self.beta  = math.sin(theta) * math.sin(phi)
        self.gamma = math.cos(theta)

        self.h = self.slider_h.value() / 100.0


    def update_inverse_kinematics(self):
        """
        Solve robot angles from user input
        """

        self.compute_vector()

        # Solve IK
        self.robot.solve_inverse_kinematics_vector(
            self.alpha, self.beta, self.gamma, self.h
        )


    def update_plot(self):
        """
        Update 3D visualization with latest robot state
        """

        # Get points from robot model
        A_points = np.array([self.robot.A1, self.robot.A2, self.robot.A3])
        B_points = np.array([self.robot.B1, self.robot.B2, self.robot.B3])
        C_points = np.array([self.robot.C1, self.robot.C2, self.robot.C3])

        # Update scatter plots
        self.scatter_A.setData(pos=A_points)
        self.scatter_B.setData(pos=B_points)
        self.scatter_C.setData(pos=C_points)

        # Update lines
        self.line_A.setData(pos=[*A_points, A_points[0]])
        self.line_B.setData(pos=[*B_points, B_points[0]])

        self.line_C1.setData(pos=[self.robot.A1, self.robot.C1, self.robot.B1])
        self.line_C2.setData(pos=[self.robot.A2, self.robot.C2, self.robot.B2])
        self.line_C3.setData(pos=[self.robot.A3, self.robot.C3, self.robot.B3])


# =========================================================
# MAIN PROGRAM
# =========================================================

if __name__ == '__main__':

    app = QtWidgets.QApplication(sys.argv)

    # 🔧 CHANGE THESE → robot geometry
    robot = RobotKinematics(
        lp=7.125,
        l1=6.2,
        l2=4.5,
        lb=4.0,
        invert=False
    )

    window = MainWindow(robot)
    window.show()

    sys.exit(app.exec_())