import numpy as np
from utils.visualization import plot_validation_result
from utils.robot_model import create_robot

def validate_points(joint_angles, intended_poses):
    """Validate points by comparing FK results with intended poses"""
    robot = create_robot()
    
    actual_poses = []
    for q in joint_angles:
        T = robot.fkine(q)
        actual_poses.append(T)
    
    actual_points = np.array([T.t for T in actual_poses])
    intended_points = np.array([T.t for T in intended_poses])
    
    # Calculate errors
    pos_errors = np.linalg.norm(actual_points - intended_points, axis=1) * 1000  # mm
    print(f"\nValidation Results:")
    print(f"Position Error (mm) - Max: {np.max(pos_errors):.2f}, Mean: {np.mean(pos_errors):.2f}")
    
    plot_validation_result(actual_points, intended_points)
    
    return actual_poses, pos_errors
