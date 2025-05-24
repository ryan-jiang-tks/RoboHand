import numpy as np
import matplotlib.pyplot as plt
import os
import csv
from datetime import datetime
from math import cos, sin, pi, atan2, acos, asin

class PUMA560Trajectory:
    def __init__(self):
        # PUMA 560 Standard DH parameters: [theta, d, a, alpha]
        self.dh = np.array([
            [0,      0.660,      0,      90],      # theta1, d1, a1, alpha1
            [0,      0,        0.432,    180],      # theta2, d2, a2, alpha2
            [0,      0.15,     0.020,    90],      # theta3, d3, a3, alpha3
            [0,      -0.4318,  0,        90],      # theta4, d4, a4, alpha4
            [0,      0,        0,        90],      # theta5, d5, a5, alpha5
            [0,      -0.056,   0,       -180]      # theta6, d6, a6, alpha6
        ], dtype=np.float64)
        
        # Joint limits in degrees
        self.joint_limits = [
            (-360, 360),    # deg
            (-360, 360), 
            (-360, 360), 
            (-360, 360), 
            (-360, 360), 
            (-360, 360)
        ]

    def dh_matrix(self, theta, d, a, alpha):
        """Standard DH transformation matrix with parameters in order: theta, d, a, alpha
        theta and alpha should be in degrees"""
        # Convert angles to radians for calculation
        return np.array([
            [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
            [0, sin(alpha), cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, joints_deg):
        """
        Calculate forward kinematics
        joints_deg: joint angles in degrees
        """
        T = np.eye(4)
        for i in range(6):
            theta = joints_deg[i] + self.dh[i][0]  # Add joint offset (both in degrees)
            d = self.dh[i][1]      # d parameter
            a = self.dh[i][2]      # a parameter
            alpha = self.dh[i][3]  # alpha in degrees
            Ti = self.dh_matrix(theta, d, a, alpha)
            T = T @ Ti
        return T

    def inverse_kinematics(self, T_target, config='righty_up_noflip'):
        """Standard inverse kinematics for PUMA560
        T_target: 4x4 homogeneous transformation matrix
        config: robot configuration 'righty_up_noflip' (default), 'lefty_up_noflip',
               'righty_down_noflip', 'lefty_down_noflip', etc.
        Returns: 6 joint angles in degrees"""
        
        # Extract parameters from DH table
        d1 = self.dh[0, 1]  # 0.660
        a2 = self.dh[1, 2]  # 0.432
        a3 = self.dh[2, 2]  # 0.020
        d4 = self.dh[3, 1]  # -0.4318
        d6 = self.dh[5, 1]  # -0.056

        # Parse configuration
        righty = 'righty' in config
        elbow_up = 'up' in config
        wrist_flip = 'flip' in config
        
        # Extract position and rotation from target matrix
        o = T_target[0:3, 3]  # Position vector
        R = T_target[0:3, 0:3]  # Rotation matrix
        
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
        
        if elbow_up:
            theta2 = alpha + beta
        else:
            theta2 = alpha - beta
        
        cos_theta3 = (D**2 - a2**2 - (a3**2 + d4**2)) / (2 * a2 * np.sqrt(a3**2 + d4**2))
        cos_theta3 = np.clip(cos_theta3, -1, 1)
        theta3 = atan2(a3, d4) - acos(cos_theta3)
        if not elbow_up:
            theta3 = -theta3
        
        # Calculate R0_3 for wrist angles
        R0_3 = self.get_R0_3(theta1, theta2, theta3)
        R3_6 = np.linalg.inv(R0_3) @ R
        
        # Theta4, Theta5, Theta6: Wrist angles
        theta4 = atan2(R3_6[1, 2], R3_6[0, 2])
        theta5 = acos(R3_6[2, 2])
        if wrist_flip:
            theta4 += pi
            theta5 = -theta5
        theta6 = atan2(R3_6[2, 1], -R3_6[2, 0])
        
        # Convert to degrees and return
        return np.degrees([theta1, theta2, theta3, theta4, theta5, theta6])

    def get_R0_3(self, theta1, theta2, theta3):
        """Calculate rotation matrix from base to joint 3"""
        # Calculate individual transformation matrices
        T1 = self.dh_matrix(degrees(theta1), self.dh[0, 1], self.dh[0, 2], self.dh[0, 3])
        T2 = self.dh_matrix(degrees(theta2), self.dh[1, 1], self.dh[1, 2], self.dh[1, 3])
        T3 = self.dh_matrix(degrees(theta3), self.dh[2, 1], self.dh[2, 2], self.dh[2, 3])
        
        # Combine transformations and extract rotation matrix
        T0_3 = T1 @ T2 @ T3
        return T0_3[0:3, 0:3]

    def joint_space_trajectory(self, q_start_deg, q_end_deg, tf, dt=0.01):
        """
        Cubic polynomial trajectory in joint space
        Input angles in degrees
        Returns angles in degrees
        """
        t = np.arange(0, tf, dt)
        
        # Convert to radians for calculation
        q_start = np.radians(q_start_deg)
        q_end = np.radians(q_end_deg)
        
        a0 = q_start
        a1 = np.zeros(6)
        a2 = 3/(tf**2)*(q_end - q_start)
        a3 = -2/(tf**3)*(q_end - q_start)
        
        # Calculate trajectories in radians
        q_rad = a0 + a1*t[:, np.newaxis] + a2*t[:, np.newaxis]**2 + a3*t[:, np.newaxis]**3
        qd_rad = a1 + 2*a2*t[:, np.newaxis] + 3*a3*t[:, np.newaxis]**2
        qdd_rad = 2*a2 + 6*a3*t[:, np.newaxis]
        
        # Convert back to degrees for output
        q = np.degrees(q_rad)
        qd = np.degrees(qd_rad)  # convert velocity to deg/s
        qdd = np.degrees(qdd_rad)  # convert acceleration to deg/s²
        
        return q, qd, qdd, t

    def save_trajectory_to_csv(self, q, qd, qdd, t):
        """Save trajectory data to CSV file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join("data", f"trajectory_{timestamp}.csv")
        
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Write data with time in first column
            for i in range(len(t)):  # All rows except last
                row = [t[i]]  # Time first
                for j in range(6):
                    row.extend([q[i,j], qd[i,j], qdd[i,j]])
                writer.writerow(row)
        return filename

    def cartesian_space_trajectory(self, p_start, p_end, tf, dt=0.01):
        """Linear trajectory in Cartesian space
        Returns: Cartesian positions and timestamps"""
        t = np.arange(0, tf, dt)
        x = np.linspace(p_start[0], p_end[0], len(t))
        y = np.linspace(p_start[1], p_end[1], len(t))
        z = np.linspace(p_start[2], p_end[2], len(t))
        return np.vstack([x, y, z]).T, t

    def cartesian_to_joint_trajectory(self, cartesian_points, t, orientation=np.eye(3)):
        """Convert Cartesian trajectory to joint space trajectory
        cartesian_points: Nx3 array of [x,y,z] positions
        t: timestamps
        orientation: 3x3 rotation matrix for end-effector orientation
        Returns: joint angles, velocities, accelerations and timestamps"""
        n_points = len(cartesian_points)
        q = np.zeros((n_points, 6))
        qd = np.zeros((n_points, 6))
        qdd = np.zeros((n_points, 6))
        dt = t[1] - t[0]

        # Convert each Cartesian point to joint angles
        for i in range(n_points):
            # Create transformation matrix for current point
            T = np.eye(4)
            T[0:3, 0:3] = orientation
            T[0:3, 3] = cartesian_points[i]
            
            # Calculate inverse kinematics
            q[i] = self.inverse_kinematics(T)
        
        # Calculate velocities and accelerations using finite differences
        # Velocity (central difference)
        qd[1:-1] = (q[2:] - q[:-2]) / (2 * dt)
        qd[0] = (q[1] - q[0]) / dt  # Forward difference for first point
        qd[-1] = (q[-1] - q[-2]) / dt  # Backward difference for last point
        
        # Acceleration (central difference)
        qdd[1:-1] = (qd[2:] - qd[:-2]) / (2 * dt)
        qdd[0] = (qd[1] - qd[0]) / dt  # Forward difference for first point
        qdd[-1] = (qd[-1] - qd[-2]) / dt  # Backward difference for last point
        
        return q, qd, qdd, t

    # def star_trajectory(self, center, size, height, tf, dt=0.01):
    #     """Generate trajectory for drawing a star
    #     Returns: joint angles for the complete star trajectory"""
    #     # Get star points
    #     points = self.generate_star_points(center, size, height)
    #     self.plot_star_points(points)
        
    #     # Generate trajectory
    #     t_segment = tf / len(points)
        
    #     # Initialize arrays for complete trajectory
    #     all_q = []
    #     all_qd = []
    #     all_qdd = []
    #     all_t = []
        
    #     # Generate trajectories between consecutive points
    #     for i in range(len(points)-1):
    #         # Generate Cartesian trajectory segment
    #         cart_points, t = self.cartesian_space_trajectory(points[i], points[i+1], t_segment, dt)
            
    #         # Convert to joint space trajectory
    #         q, qd, qdd, t = self.cartesian_to_joint_trajectory(cart_points, t)
            
    #         # Accumulate trajectories
    #         if i == 0:
    #             all_q = q
    #             all_qd = qd
    #             all_qdd = qdd
    #             all_t = t
    #         else:
    #             all_q = np.vstack((all_q, q[1:]))
    #             all_qd = np.vstack((all_qd, qd[1:]))
    #             all_qdd = np.vstack((all_qdd, qdd[1:]))
    #             all_t = np.concatenate((all_t, t[1:] + all_t[-1]))
        
    #     # Validate the trajectory
    #     print("\nValidating trajectory...")
    #     actual_points= self.validate_trajectory(all_q, points)
        
    #     return all_q, all_qd, all_qdd, all_t

    def generate_star_points(self, center, size, height, num_points=20):
        """Generate points for a 5-pointed star in Cartesian space
        center: [x, y, z] center position
        size: radius of the outer points
        height: z-height of the star
        num_points: number of points per segment
        Returns: numpy array of points forming a star"""
        
        # Define 10 angles for the star (5 outer points and 5 inner points)
        angles = np.linspace(0, 2*np.pi, 11)[:-1]  # 10 points, removing the duplicate last point
        # Rotate by -pi/2 to start from top point
        angles = angles - np.pi/2
        
        # Inner radius (golden ratio)
        inner_size = size * 0.382
        
        # Initialize points array
        points = []
        
        # Generate the main vertices of the star
        vertices = []
        for i in range(10):  # 10 points total (5 outer, 5 inner)
            r = size if i % 2 == 0 else inner_size  # Alternate between outer and inner radius
            x = center[0] + r * np.cos(angles[i])
            y = center[1] + r * np.sin(angles[i])
            vertices.append([x, y, height])
        
        # Add the first vertex again to close the star
        vertices.append(vertices[0])
        vertices = np.array(vertices)
        
        # Create smooth segments between vertices
        for i in range(len(vertices) - 1):
            t = np.linspace(0, 1, num_points)
            segment = np.array([
                np.linspace(vertices[i][0], vertices[i+1][0], num_points),
                np.linspace(vertices[i][1], vertices[i+1][1], num_points),
                np.full(num_points, height)
            ]).T
            points.extend(segment)
    
        return np.array(points)

    def validate_trajectory(self, joint_angles, intended_points):
        """Validate trajectory by comparing forward kinematics results with intended points
        joint_angles: Array of joint angles (degrees)
        intended_points: Array of intended Cartesian positions"""
        
        # Calculate actual end-effector positions using forward kinematics
        actual_points = []
        for q in joint_angles:
            T = self.forward_kinematics(q)
            actual_points.append(T[0:3, 3])  # Extract position from transformation matrix
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

    def plot_trajectory(self, q_traj, t):
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

    def plot_star_points(self, points):
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

if __name__ == "__main__":
    puma = PUMA560Trajectory()
    
    # # Example joint angles in degrees (home position)
    # q_start = np.array([0, 60, 60, 0, 0, 0])
    # q_end = np.array([90, -60, 45, 0, 30, 0])
    
    # # Generate trajectory
    # q_traj, qd_traj, qdd_traj, t = puma.joint_space_trajectory(q_start, q_end, 5)
    
    # # Save trajectory data to CSV
    # csv_file = puma.save_trajectory_to_csv(q_traj, qd_traj, qdd_traj, t)
    # print(f"Trajectory data saved to: {csv_file}")
    
    # # Plot results (already in degrees)
    # puma.plot_trajectory(q_traj, t)
    
    # Define star parameters
    center = [0.4, 0.2, 0.5]  # Center position in meters
    size = 0.2  # Star size in meters
    height = 0.6  # Height in meters
    
    # Generate star trajectory
    q_traj_star, qd_traj_star, qdd_traj_star, t_star = puma.star_traImportError: attempted relative import beyond top-level packagejectory(center, size, height, 10)  # 10 seconds total
    
    # Save trajectory data to CSV
    csv_file_star = puma.save_trajectory_to_csv(q_traj_star, qd_traj_star, qdd_traj_star, t_star)
    print(f"Star trajectory data saved to: {csv_file_star}")
    
    # Plot star trajectory results
    puma.plot_trajectory(q_traj_star, t_star)
