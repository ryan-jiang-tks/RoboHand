import numpy as np
from math import cos, sin, radians ,degrees
from .dh_params import PUMA560_DH_PARAMS as dh

def dh_matrix(theta, d, a, alpha):
    """Standard DH transformation matrix with parameters in order: theta, d, a, alpha
    theta and alpha should be in degrees"""
    # Convert angles to radians for calculation
    return np.array([
        [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

def get_R0_3(theta1, theta2, theta3,dh = dh):
    """Calculate rotation matrix from base to joint 3"""
    # Calculate individual transformation matrices
    T1 = dh_matrix(degrees(theta1), dh[0, 1], dh[0, 2], dh[0, 3])
    T2 = dh_matrix(degrees(theta2), dh[1, 1], dh[1, 2], dh[1, 3])
    T3 = dh_matrix(degrees(theta3), dh[2, 1], dh[2, 2], dh[2, 3])
    
    # Combine transformations and extract rotation matrix
    T0_3 = T1 @ T2 @ T3
    return T0_3[0:3, 0:3]