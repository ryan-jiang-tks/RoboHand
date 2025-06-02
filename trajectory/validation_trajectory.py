import numpy as np
from roboticstoolbox import ctraj
from spatialmath import SE3
from utils.visualization import plot_validation_result
from utils.robot_model import create_robot
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

def check_smoothness(t, q, qd, qdd, threshold_vel=200, threshold_acc=10000):
    """Check trajectory smoothness and identify sharp changes"""
    # Calculate jerk
    dt = t[1] - t[0]
    jerk = np.gradient(qdd, dt, axis=0)
    
    # Find sharp changes
    vel_spikes = np.where(np.abs(qd) > threshold_vel)
    acc_spikes = np.where(np.abs(qdd) > threshold_acc)
    
    return vel_spikes, acc_spikes, jerk

def plot_dynamics(t, q, qd, qdd, vel_spikes=None, acc_spikes=None):
    """Plot position, velocity, and acceleration profiles"""
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 12))
    
    # Position
    for i in range(6):
        ax1.plot(t, q[:, i], label=f'Joint {i+1}')
    ax1.set_ylabel('Position (rad)')
    ax1.grid(True)
    ax1.legend()
    
    # Velocity
    for i in range(6):
        ax2.plot(t, qd[:, i], label=f'Joint {i+1}')
        if vel_spikes is not None:
            ax2.scatter(t[vel_spikes[0][vel_spikes[1] == i]], 
                       qd[vel_spikes[0][vel_spikes[1] == i], i], 
                       color='red', marker='x')
    ax2.set_ylabel('Velocity (rad/s)')
    ax2.grid(True)
    
    # Acceleration
    for i in range(6):
        ax3.plot(t, qdd[:, i], label=f'Joint {i+1}')
        if acc_spikes is not None:
            ax3.scatter(t[acc_spikes[0][acc_spikes[1] == i]], 
                       qdd[acc_spikes[0][acc_spikes[1] == i], i], 
                       color='red', marker='x')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Acceleration (rad/s²)')
    ax3.grid(True)
    
    plt.tight_layout()
    plt.show()

def smooth_trajectory(t, q, qd, qdd, vel_spikes, acc_spikes, window=21):
    """Smooth trajectory around spike points using Savitzky-Golay filter"""
    q_smooth = q.copy()
    problem_indices = np.unique(np.concatenate([vel_spikes[0], acc_spikes[0]]))
    
    if len(problem_indices) == 0:
        return q, qd, qdd
    
    print(f"\nSmoothing trajectory at {len(problem_indices)} locations...")
    
    # Apply smoothing around each spike
    for idx in problem_indices:
        start_idx = max(0, idx - window//2)
        end_idx = min(len(q), idx + window//2)
        
        # Apply Savitzky-Golay filter to smooth the trajectory
        for joint in range(6):
            q_smooth[start_idx:end_idx, joint] = savgol_filter(
                q[start_idx:end_idx, joint], 
                min(window, end_idx - start_idx), 
                3
            )
    
    # Recalculate velocities and accelerations
    dt = t[1] - t[0]
    qd_smooth = np.gradient(q_smooth, dt, axis=0)
    qdd_smooth = np.gradient(qd_smooth, dt, axis=0)
    
    return q_smooth, qd_smooth, qdd_smooth

def interpolate_joint_trajectory(q, t, problem_indices, points_to_add=5):
    """Add interpolated joint values around problematic points"""
    new_q = []
    last_idx = 0
    
    for idx in problem_indices:
        if idx > 0 and idx < len(q) - 1:
            # Add points before problem area
            new_q.extend(q[last_idx:idx])
            
            # Create interpolated points
            t_local = np.linspace(0, 1, points_to_add)
            for alpha in t_local[1:-1]:  # Exclude endpoints to avoid duplicates
                q_interp = (1-alpha) * q[idx-1] + alpha * q[idx+1]
                new_q.append(q_interp)
            
        last_idx = idx + 1
    
    # Add remaining points
    new_q.extend(q[last_idx:])
    new_q = np.array(new_q)
    
    # Recalculate time vector
    tf = t[-1]  # Preserve total time
    t_new = np.linspace(0, tf, len(new_q))
    
    return new_q, t_new

def validate_trajectory(joint_angles, qd, qdd, t, intended_poses):
    """Validate trajectory including dynamics and fix if needed"""
    # Check position accuracy
    robot = create_robot()
    actual_poses = []
    for q in joint_angles:
        T = robot.fkine(q)
        actual_poses.append(T)
    
    # Check trajectory smoothness
    vel_spikes, acc_spikes, jerk = check_smoothness(t, joint_angles, qd, qdd)
    
    # Print validation results
    print("\nTrajectory Validation Results:")
    print(f"Found {len(vel_spikes[0])} velocity spikes")
    print(f"Found {len(acc_spikes[0])} acceleration spikes")
    print(f"Maximum jerk: {np.max(np.abs(jerk))} rad/s³")
    
    # Plot position accuracy
    actual_points = np.array([T.t for T in actual_poses])
    intended_points = np.array([T.t for T in intended_poses])
    plot_validation_result(actual_points, intended_points)
    
    # Plot dynamics
    plot_dynamics(t, joint_angles, qd, qdd, vel_spikes, acc_spikes)
    
    # # If spikes detected, smooth the trajectory
    # if len(vel_spikes[0]) > 0 or len(acc_spikes[0]) > 0:
    #     print("\nSpikes detected, attempting to smooth trajectory...")
    #     q_smooth, qd_smooth, qdd_smooth = smooth_trajectory(
    #         t, joint_angles, qd, qdd, vel_spikes, acc_spikes
    #     )
        
    #     # Validate smoothed trajectory
    #     print("\nValidating smoothed trajectory:")
    #     vel_spikes_new, acc_spikes_new, jerk_new = check_smoothness(
    #         t, q_smooth, qd_smooth, qdd_smooth
    #     )
        
    #     # Compare results
    #     print("\nSmoothing results:")
    #     print(f"Velocity spikes: {len(vel_spikes[0])} -> {len(vel_spikes_new[0])}")
    #     print(f"Acceleration spikes: {len(acc_spikes[0])} -> {len(acc_spikes_new[0])}")
    #     print(f"Max jerk: {np.max(np.abs(jerk)):.2f} -> {np.max(np.abs(jerk_new)):.2f}")
        
    #     # Plot comparison
    #     fig, axes = plt.subplots(2, 1, figsize=(12, 12))
    #     plot_dynamics(t, joint_angles, qd, qdd, vel_spikes, acc_spikes)
    #     axes[0].set_title("Original Trajectory")
    #     plot_dynamics(t, q_smooth, qd_smooth, qdd_smooth, vel_spikes_new, acc_spikes_new)
    #     axes[1].set_title("Smoothed Trajectory")
    #     plt.tight_layout()
    #     plt.show()
        
    #     return q_smooth, qd_smooth, qdd_smooth
    
    # return joint_angles, qd, qdd

    # If spikes detected, add points and smooth
    if len(vel_spikes[0]) > 0 or len(acc_spikes[0]) > 0:
        print("\nSpikes detected, adding interpolated points...")
        
        # # Get unique problem locations
        # problem_indices = np.unique(np.concatenate([vel_spikes[0], acc_spikes[0]]))
        
        # # Add interpolated points
        # q_new, t_new = interpolate_joint_trajectory(joint_angles, t, problem_indices)
        
        # # Recalculate velocities and accelerations
        # dt_new = t_new[1] - t_new[0]
        # qd_new = np.gradient(q_new, dt_new, axis=0)
        # qdd_new = np.gradient(qd_new, dt_new, axis=0)
        
        t_new = t
        q_new = joint_angles.copy()
        qd_new = qd.copy()
        qdd_new = qdd.copy()

        # Apply smoothing to interpolated trajectory
        q_smooth, qd_smooth, qdd_smooth = smooth_trajectory(
            t_new, q_new, qd_new, qdd_new, vel_spikes, acc_spikes
        )
        
        # Validate improved trajectory
        print("\nValidating improved trajectory:")
        vel_spikes_new, acc_spikes_new, jerk_new = check_smoothness(
            t_new, q_smooth, qd_smooth, qdd_smooth
        )
        
        # Compare results
        print("\nTrajectory improvement results:")
        print(f"Number of points: {len(t)} -> {len(t_new)}")
        print(f"Velocity spikes: {len(vel_spikes[0])} -> {len(vel_spikes_new[0])}")
        print(f"Acceleration spikes: {len(acc_spikes[0])} -> {len(acc_spikes_new[0])}")
        print(f"Max jerk: {np.max(np.abs(jerk)):.2f} -> {np.max(np.abs(jerk_new)):.2f}")
        
        return q_smooth, qd_smooth, qdd_smooth, t_new
    
    return joint_angles, qd, qdd, t
