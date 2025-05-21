import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi, atan2, acos, asin

class PUMA560Trajectory:
    def __init__(self):
        # PUMA 560 DH parameters (modified Denavit-Hartenberg)
        self.dh = np.array([
            [0, 0, 0, 0],       # theta1, d1, a1, alpha1
            [-pi/2, 0.2435, 0, -pi/2],  
            [0, -0.0934, 0.4318, 0],
            [pi/2, 0.4331, 0.0203, -pi/2],
            [-pi/2, 0, 0, pi/2],
            [0, 0, 0, 0]
        ], dtype=np.float64)
        
        self.joint_limits = [
            (-160, 160),    # deg
            (-225, 45),
            (-45, 225),
            (-110, 170),
            (-100, 100),
            (-266, 266)
        ]

    def dh_matrix(self, theta, d, a, alpha):
        return np.array([
            [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
            [0, sin(alpha), cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, joints):
        T = np.eye(4)
        for i in range(6):
            theta = joints[i] + self.dh[i][0]
            d = self.dh[i][1]
            a = self.dh[i][2]
            alpha = self.dh[i][3]
            Ti = self.dh_matrix(theta, d, a, alpha)
            T = T @ Ti
        return T

    def inverse_kinematics(self, T_target):
        # PUMA-specific inverse kinematics implementation
        # (Detailed implementation would go here)
        pass

    def joint_space_trajectory(self, q_start, q_end, tf, dt=0.01):
        """Cubic polynomial trajectory in joint space"""
        t = np.arange(0, tf, dt)
        a0 = q_start
        a1 = np.zeros(6)
        a2 = 3/(tf**2)*(q_end - q_start)
        a3 = -2/(tf**3)*(q_end - q_start)
        
        q = a0 + a1*t + a2*t**2 + a3*t**3
        qd = a1 + 2*a2*t + 3*a3*t**2
        return q, qd, t

    def cartesian_space_trajectory(self, p_start, p_end, tf, dt=0.01):
        """Linear trajectory in Cartesian space"""
        t = np.arange(0, tf, dt)
        x = np.linspace(p_start[0], p_end[0], len(t))
        y = np.linspace(p_start[1], p_end[1], len(t))
        z = np.linspace(p_start[2], p_end[2], len(t))
        return np.vstack([x, y, z]).T, t

    def plot_trajectory(self, q, t):
        plt.figure(figsize=(10,6))
        for i in range(6):
            plt.plot(t, np.degrees(q[:,i]), 
                     label=f'Joint {i+1}')
        plt.xlabel('Time (s)')
        plt.ylabel('Joint Angle (deg)')
        plt.title('Joint Space Trajectory')
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    puma = PUMA560Trajectory()
    
    # Example joint angles (home position)
    q_start = np.array([0, -pi/2, pi/2, 0, 0, 0])
    q_end = np.array([pi/2, -pi/3, pi/4, 0, pi/6, 0])
    
    # Generate trajectory
    q_traj, qd_traj, t = puma.joint_space_trajectory(q_start, q_end, 5)
    
    # Plot results
    puma.plot_trajectory(q_traj, t)
