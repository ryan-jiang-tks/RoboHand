import numpy as np
from math import pi, atan2, acos, degrees
from utils.transformations import get_R0_3
from utils.dh_params import PUMA560_DH_PARAMS as dh

def inverse_kinematics(T_target,dh_params = dh, config='righty_up_noflip'):
    """Standard inverse kinematics for PUMA560
    Args:
        T_target: 4x4 homogeneous transformation matrix
        dh_params: DH parameters array [theta, d, a, alpha]
        config: robot configuration string
    Returns:
        array of 6 joint angles in degrees
    """
    # Extract DH parameters
    d1 = dh[0, 1]  # 0.660
    a2 = dh[1, 2]  # 0.432
    a3 = dh[2, 2]  # 0.020
    d4 = dh[3, 1]  # -0.4318
    d6 = dh[5, 1]  # -0.056

    # Parse configuration
    righty = 'righty' in config
    elbow_up = 'up' in config
    wrist_flip = 'flip' in config
    
    # Extract position and rotation
    o = T_target[0:3, 3]
    R = T_target[0:3, 0:3]
    
    # Wrist center position
    oc = o - d6 * R[:, 2]
    
    # Theta1: Base rotation
    theta1 = atan2(oc[1], oc[0])
    if not righty:
        theta1 = theta1 + pi if theta1 < 0 else theta1 - pi
    
    # Theta2 & Theta3: Arm and elbow angles
    r = np.sqrt(oc[0]**2 + oc[1]**2)
    s = oc[2] - d1
    
    # Triangle sides for elbow calculation
    D = np.sqrt(r**2 + s**2)
    cos_beta = (a2**2 + D**2 - (a3**2 + d4**2)) / (2 * a2 * D)
    cos_beta = np.clip(cos_beta, -1, 1)
    
    beta = acos(cos_beta)
    alpha = atan2(s, r)
    
    theta2 = alpha + beta if elbow_up else alpha - beta
    
    cos_theta3 = (D**2 - a2**2 - (a3**2 + d4**2)) / (2 * a2 * np.sqrt(a3**2 + d4**2))
    cos_theta3 = np.clip(cos_theta3, -1, 1)
    theta3 = atan2(a3, d4) - acos(cos_theta3)
    if not elbow_up:
        theta3 = -theta3
    
    # Calculate wrist angles
    R0_3 = get_R0_3(theta1, theta2, theta3,dh)
    R3_6 = np.linalg.inv(R0_3) @ R
    
    theta4 = atan2(R3_6[1, 2], R3_6[0, 2])
    theta5 = acos(R3_6[2, 2])
    if wrist_flip:
        theta4 += pi
        theta5 = -theta5
    theta6 = atan2(R3_6[2, 1], -R3_6[2, 0])
    
    return np.degrees([theta1, theta2, theta3, theta4, theta5, theta6])
