import numpy as np
from utils.visualization import plot_star_points
from .cartesian_trajectory import cartesian_to_joint_trajectory, cartesian_space_trajectory
from .validation_trajectory import validate_trajectory
import roboticstoolbox as rtb

def generate_star_points(center, size, height, num_points=20):
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

def star_trajectory(center, size, height, tf, dt=0.01):
        """Generate trajectory for drawing a star
        Returns: joint angles for the complete star trajectory"""
        # Get star points
        points = generate_star_points(center, size, height)
        plot_star_points(points)
        
        # Generate trajectory
        t_segment = tf / len(points)
        
        # Initialize arrays for complete trajectory
        all_q = []
        all_qd = []
        all_qdd = []
        all_t = []
        
        # Generate trajectories between consecutive points
        for i in range(len(points)-1):
            # Generate Cartesian trajectory segment
            cart_points, t = cartesian_space_trajectory(points[i], points[i+1], t_segment, dt)
            
            # Convert to joint space trajectory
            q, qd, qdd, t = cartesian_to_joint_trajectory(cart_points, t)
            
            # Accumulate trajectories
            if i == 0:
                all_q = q
                all_qd = qd
                all_qdd = qdd
                all_t = t
            else:
                all_q = np.vstack((all_q, q[1:]))
                all_qd = np.vstack((all_qd, qd[1:]))
                all_qdd = np.vstack((all_qdd, qdd[1:]))
                all_t = np.concatenate((all_t, t[1:] + all_t[-1]))
        
        # Validate the trajectory
        print("\nValidating trajectory...")
        actual_points= validate_trajectory(all_q, points)
        
        return all_q, all_qd, all_qdd, all_t