import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb
from utils.visualization import plot_star_points
from utils.robot_model import create_robot
from points.star_points import generate_star_points
from points.points_validation import validate_points
from trajectory.validation_trajectory import validate_trajectory

def star_trajectory(center, size, height, tf, dt=0.01):
    """Generate star trajectory with SE3 poses"""
    # Generate star points as SE3 poses
    poses = generate_star_points(center, size, height, 3)
    samples = int(tf / dt)
    # Extract positions for visualization
    positions = np.array([pose.t for pose in poses])
    plot_star_points(positions)
    
    # Initialize trajectory arrays
    all_q = []
    all_qd = []
    all_qdd = []
    robot = create_robot()
    
    # Generate joint trajectories through poses
    q0 = np.zeros(6)  # Initial joint configuration
    for i in range(len(poses)-1):
        # Generate Cartesian trajectory between poses
        traj = rtb.ctraj(poses[i], poses[i+1], int(samples/len(poses)))
        
        # Convert to joint space
        qsol = []
        for T in traj:
            sol = robot.ikine_LM(T, q0=q0)
            if sol.success:
                qsol.append(sol.q)
                q0 = sol.q  # Use previous solution as initial guess
        all_q.extend(qsol)

        
    # Validate the trajectory
    print("\nValidating trajectory...")
    actual_poses, errors = validate_trajectory(all_q, poses)

    # Convert to numpy array
    all_q = np.array(np.degrees(all_q))
    t = np.linspace(0, tf, len(all_q))
    
    # Calculate velocities and accelerations
    dt = t[1] - t[0]
    all_qd = np.gradient(all_q, dt, axis=0)
    all_qdd = np.gradient(all_qd, dt, axis=0)
    
  
    
    # # If errors are too large, try to optimize
    # if max(pos_errors) > 5.0:  # 5mm threshold
    #     print("Large errors detected, attempting optimization...")
    #     # Try different IK solution
    #     q0 = np.zeros(6)
    #     for i, pose in enumerate(poses):
    #         sol = robot.ikine_min(pose, q0)
    #         if sol.success:
    #             all_q[i] = sol.q
    #             q0 = sol.q
    
    return all_q, all_qd, all_qdd, t