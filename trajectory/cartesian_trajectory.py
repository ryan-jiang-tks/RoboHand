import numpy as np
from kinematics.inverse_kinematics import inverse_kinematics, create_robot

def cartesian_space_trajectory( p_start, p_end, tf, dt=0.01):
        """Linear trajectory in Cartesian space
        Returns: Cartesian positions and timestamps"""
        t = np.arange(0, tf, dt)
        x = np.linspace(p_start[0], p_end[0], len(t))
        y = np.linspace(p_start[1], p_end[1], len(t))
        z = np.linspace(p_start[2], p_end[2], len(t))
        return np.vstack([x, y, z]).T, t

def cartesian_to_joint_trajectory(ctraj):
        """Convert Cartesian trajectory to joint space trajectory
        cartesian_points: Nx3 array of [x,y,z] positions
        t: timestamps
        orientation: 3x3 rotation matrix for end-effector orientation
        # Returns: joint angles, velocities, accelerations and timestamps"""
        # n_points = len(cartesian_points)
        # q = np.zeros((n_points, 6))
        # qd = np.zeros((n_points, 6))
        # qdd = np.zeros((n_points, 6))
        # dt = t[1] - t[0]
        robot = create_robot()
        q_traj=[]
        for T in ctraj:
                 sol = robot.ikine_LM(T, q0=q0)
        if sol.success:
                q_traj.append(sol.q)
                q0 = sol.q
        else:
                q_traj.append(q0)

        q=q_traj
        # # Calculate velocities and accelerations using finite differences
        # # Velocity (central difference)
        # qd[1:-1] = (q[2:] - q[:-2]) / (2 * dt)
        # qd[0] = (q[1] - q[0]) / dt  # Forward difference for first point
        # qd[-1] = (q[-1] - q[-2]) / dt  # Backward difference for last point
        
        # # Acceleration (central difference)
        # qdd[1:-1] = (qd[2:] - qd[:-2]) / (2 * dt)
        # qdd[0] = (qd[1] - qd[0]) / dt  # Forward difference for first point
        # qdd[-1] = (qd[-1] - qd[-2]) / dt  # Backward difference for last point
        
        return q, qd, qdd, t