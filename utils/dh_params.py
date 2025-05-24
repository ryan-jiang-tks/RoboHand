import numpy as np

# PUMA 560 Standard DH parameters: [theta, d, a, alpha]
PUMA560_DH_PARAMS = np.array([
    [0,      0.660,      0,      90],      # theta1, d1, a1, alpha1
    [0,      0,        0.432,    180],     # theta2, d2, a2, alpha2
    [0,      0.15,     0.020,    90],      # theta3, d3, a3, alpha3
    [0,      -0.4318,  0,        90],      # theta4, d4, a4, alpha4
    [0,      0,        0,        90],      # theta5, d5, a5, alpha5
    [0,      -0.056,   0,       -180]      # theta6, d6, a6, alpha6
], dtype=np.float64)

# Joint limits in degrees
JOINT_LIMITS = [
    (-360, 360),    # Joint 1
    (-360, 360),    # Joint 2
    (-360, 360),    # Joint 3
    (-360, 360),    # Joint 4
    (-360, 360),    # Joint 5
    (-360, 360)     # Joint 6
]
