import numpy as np
from math import cos, sin, radians, degrees
from .dh_params import PUMA560_DH_PARAMS as dh

def dh_matrix(theta, d, a, alpha):
    """Standard DH transformation matrix with parameters in order: theta, d, a, alpha
    theta and alpha should be in degrees"""
    # Convert angles to radians for calculation
    theta_rad = theta
    alpha_rad = alpha
    # theta_rad = radians(theta)
    # alpha_rad = radians(alpha)
    
    return np.array([
        [cos(theta_rad), -sin(theta_rad)*cos(alpha_rad), sin(theta_rad)*sin(alpha_rad), a*cos(theta_rad)],
        [sin(theta_rad), cos(theta_rad)*cos(alpha_rad), -cos(theta_rad)*sin(alpha_rad), a*sin(theta_rad)],
        [0, sin(alpha_rad), cos(alpha_rad), d],
        [0, 0, 0, 1]
    ])

def get_R0_3(theta1, theta2, theta3,dh_param = dh):
    """Calculate rotation matrix from base to joint 3"""
    # Calculate individual transformation matrices
    T1 = dh_matrix(theta1, dh_param[0, 1], dh_param[0, 2], dh_param[0, 3])
    T2 = dh_matrix(theta2, dh_param[1, 1], dh_param[1, 2], dh_param[1, 3])
    T3 = dh_matrix(theta3, dh_param[2, 1], dh_param[2, 2], dh_param[2, 3])
    
    # Combine transformations and extract rotation matrix
    T0_3 = T1 @ T2 @ T3
    return T0_3[0:3, 0:3]