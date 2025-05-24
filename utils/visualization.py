import matplotlib.pyplot as plt
import numpy as np
import os
from datetime import datetime

def plot_trajectory(q_traj, t):
    """Plot joint trajectories over time
    q_traj: joint angles in degrees
    t: time vector"""
    plt.figure(figsize=(10, 8))
    for i in range(6):
        plt.plot(t, q_traj[:, i], label=f'Joint {i+1}')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Joint Angle (degrees)')
    plt.title('Joint Space Trajectory')
    plt.legend()
    plt.grid(True)
    
    # Create 'plots' directory if it doesn't exist
    plots_dir = os.path.join(os.path.dirname(__file__), 'plots')
    os.makedirs(plots_dir, exist_ok=True)
    
    # Save plot with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    plt.savefig(os.path.join(plots_dir, f'trajectory_plot_{timestamp}.png'))
    plt.show()

def plot_star_points(points):
    """Plot star points in 3D space"""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the star points
    ax.plot(points[:, 0], points[:, 1], points[:, 2], 'b-', label='Star Path')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='r', marker='o')
    
    # Plot settings
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Star Trajectory in 3D Space')
    ax.legend()
    ax.grid(True)
    
    # Create 'plots' directory if it doesn't exist
    plots_dir = os.path.join(os.path.dirname(__file__), 'plots')
    os.makedirs(plots_dir, exist_ok=True)
    
    # Save plot with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    plt.savefig(os.path.join(plots_dir, f'star_3d_plot_{timestamp}.png'))
    plt.show()

def plot_validation_result(actual_points, intended_points):
    """Validate trajectory by comparing forward kinematics results with intended points
    joint_angles: Array of joint angles (degrees)
    intended_points: Array of intended Cartesian positions"""
    
    # Calculate actual end-effector positions using forward kinematics
    actual_points = np.array(actual_points)
    
    # # Calculate position error
    # position_error = np.linalg.norm(actual_points - intended_points, axis=1)
    # max_error = np.max(position_error)
    # avg_error = np.mean(position_error)
    
    # print(f"Maximum position error: {max_error*1000:.2f} mm")
    # print(f"Average position error: {avg_error*1000:.2f} mm")
    
    # Plot actual vs intended trajectory
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot both trajectories
    ax.scatter(intended_points[:, 0], intended_points[:, 1], intended_points[:, 2], 
            'b', label='Intended Path')
    ax.scatter(actual_points[:, 0], actual_points[:, 1], actual_points[:, 2], c='r', marker='o'
            ,label='Actual Path')
    
    # Plot settings
    # 设置坐标轴范围
    ax.set_xlim([0, 0.6])
    ax.set_ylim([0, 0.6])
    ax.set_zlim([0.4, 0.7])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Trajectory Validation')
    ax.legend()
    ax.grid(True)
    
    # Save validation plot
    plots_dir = os.path.join(os.path.dirname(__file__), 'plots')
    os.makedirs(plots_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    plt.savefig(os.path.join(plots_dir, f'trajectory_validation_{timestamp}.png'))
    plt.show()
    
    return actual_points
