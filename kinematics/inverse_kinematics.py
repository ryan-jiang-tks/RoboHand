import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb
from utils.dh_params import PUMA560_DH_PARAMS as dh

def create_robot():
    """Create PUMA560 robot model"""
    robot = rtb.DHRobot([
        rtb.RevoluteDH(d=dh[0,1], a=dh[0,2], alpha=dh[0,3]),
        rtb.RevoluteDH(d=dh[1,1], a=dh[1,2], alpha=dh[1,3]),
        rtb.RevoluteDH(d=dh[2,1], a=dh[2,2], alpha=dh[2,3]),
        rtb.RevoluteDH(d=dh[3,1], a=dh[3,2], alpha=dh[3,3]),
        rtb.RevoluteDH(d=dh[4,1], a=dh[4,2], alpha=dh[4,3]),
        rtb.RevoluteDH(d=dh[5,1], a=dh[5,2], alpha=dh[5,3])
    ], name="PUMA560")
    return robot

def inverse_kinematics(T_target, dh_params=dh, config='righty_up_noflip'):
    """Inverse kinematics using robotics toolbox
    Args:
        T_target: 4x4 homogeneous transformation matrix
        config: configuration string 'righty_up_noflip', etc.
    Returns:
        array of 6 joint angles in degrees
    """
    # Create robot model
    robot = create_robot()
    
    # Convert target to SE3
    T = SE3(T_target)
    
    # Parse configuration
    mask = []
    mask.append(1 if 'righty' in config else -1)  # shoulder right/left
    mask.append(1 if 'up' in config else -1)      # elbow up/down
    mask.append(1 if not 'flip' in config else -1)  # wrist not flipped/flipped
    
    # Solve IK using Levenberg-Marquardt method
    sol = robot.ikine_LM(T)
    
    if not sol.success:
        raise ValueError("Inverse kinematics failed to converge")
    
    # Convert to degrees and return
    return (sol.q)
