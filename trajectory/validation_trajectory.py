import numpy as np
from roboticstoolbox import ctraj
from spatialmath import SE3
from utils.visualization import plot_validation_result
from utils.robot_model import create_robot

def validate_trajectory(joint_angles, intended_poses):
    """Validate trajectory by comparing FK with intended poses"""
    robot = create_robot()
    
    # Calculate actual end-effector poses using forward kinematics
    actual_poses = []
    for q in joint_angles:
        T = robot.fkine(q)
        actual_poses.append(T)
    
    # # Calculate errors
    # pos_errors = []
    # orient_errors = []
    
    # for T_actual, T_intended in zip(actual_poses, intended_poses):
    #     # Position error (mm)
    #     pos_error = np.linalg.norm(T_actual.t - T_intended.t) * 1000
    #     pos_errors.append(pos_error)
        
    #     # Orientation error (degrees)
    #     orient_error = np.degrees(T_actual.angle_between(T_intended))
    #     orient_errors.append(orient_error)
    
    # # Print statistics
    # print(f"\nValidation Results:")
    # print(f"Position Error (mm) - Max: {max(pos_errors):.2f}, Mean: {np.mean(pos_errors):.2f}")
    # print(f"Orientation Error (deg) - Max: {max(orient_errors):.2f}, Mean: {np.mean(orient_errors):.2f}")
    
    # Extract positions for visualization
    actual_points = np.array([T.t for T in actual_poses])
    intended_points = np.array([T.t for T in intended_poses])
    
    # Visualize results
    plot_validation_result(actual_points, intended_points)
    
    return 0
