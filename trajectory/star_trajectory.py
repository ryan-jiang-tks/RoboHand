import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb
from utils.visualization import plot_star_points
from utils.robot_model import create_robot
from trajectory.validation_trajectory import validate_trajectory
def generate_star_points(center, size, height, num_points=10):
    """Generate star points as SE3 poses"""
    # Define angles for star points
    angles = np.linspace(0, 2*np.pi, 11)[:-1] - np.pi/2
    inner_size = size * 0.382
    
    # Generate vertices with SE3 poses
    vertices = []
    for i in range(10):
        r = size if i % 2 == 0 else inner_size
        x = center[0] + r * np.cos(angles[i])
        y = center[1] + r * np.sin(angles[i])
        z = height
        
        # Calculate orientation - pointing down and tangent to path
        Rz = SE3.Rz(angles[i])  # Rotate to face direction of motion
        Ry = SE3.Ry(np.pi)      # Point downward
        T = SE3(x, y, z) * Rz * Ry
        vertices.append(T)
    
    # Close the loop
    vertices.append(vertices[0])
    
    # Interpolate between vertices
    points = []
    for i in range(len(vertices)-1):
        ctraj = rtb.ctraj(vertices[i], vertices[i+1], num_points)
        points.extend(ctraj)
    
    return points

def star_trajectory(center, size, height, tf, dt=0.01):
    """Generate star trajectory with SE3 poses"""
    # Generate star points as SE3 poses
    poses = generate_star_points(center, size, height,3)
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
    validate_trajectory(all_q, poses)

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