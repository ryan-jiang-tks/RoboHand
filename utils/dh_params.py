import numpy as np
from math import radians, degrees, sin, cos, atan2, acos, pi
# PUMA 560 Standard DH parameters: [theta, d, a, alpha]
PUMA560_DH_PARAMS = np.array([
    [0,      0.660,      0.01,      pi/2],      # theta1, d1, a1, alpha1
    [0,      0,        0.432,    pi],     # theta2, d2, a2, alpha2
    [0,      0.15,     0.020,    pi/2],      # theta3, d3, a3, alpha3
    [0,      -0.4318,  0,        pi/2],      # theta4, d4, a4, alpha4
    [0,      0,        0,        pi/2],      # theta5, d5, a5, alpha5
    [0,      -0.056,   0,       -pi]      # theta6, d6, a6, alpha6
], dtype=np.float64)

# Joint limits in degrees
JOINT_LIMITS = [
    (-pi, pi),    # Joint 1
    (-pi, pi),    # Joint 2
    (-pi, pi),    # Joint 3
    (-pi, pi),    # Joint 4
    (-pi, pi),    # Joint 5
    (-pi, pi)     # Joint 6
]
