import numpy as np
from kinematics.forward_kinematics import forward_kinematics
from utils.visualization import plot_validation_result
from utils.dh_params import PUMA560_DH_PARAMS

def validate_trajectory(joint_angles, intended_points):
    """Validate trajectory by comparing forward kinematics results with intended points"""
    # Calculate actual end-effector positions using forward kinematics
    actual_points = []
    for q in joint_angles:
        T = forward_kinematics(q, PUMA560_DH_PARAMS)
        actual_points.append(T[0:3, 3])
    actual_points = np.array(actual_points)
    
    # Visualize the results
    plot_validation_result(actual_points,intended_points)
    
    return actual_points
