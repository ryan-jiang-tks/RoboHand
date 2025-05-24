import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from robot_model import create_puma560

def generate_star_points(center, size, height, num_points=20):
    """Generate 5-pointed star points"""
    angles = np.linspace(0, 2*np.pi, 11)[:-1]
    angles = angles - np.pi/2  # Start from top
    inner_size = size * 0.382  # Golden ratio
    points = []
    
    # Generate vertices
    vertices = []
    for i in range(10):
        r = size if i % 2 == 0 else inner_size
        x = center[0] + r * np.cos(angles[i])
        y = center[1] + r * np.sin(angles[i])
        vertices.append([x, y, height])
    
    # Close the star
    vertices.append(vertices[0])
    vertices = np.array(vertices)
    
    # Create smooth segments
    for i in range(len(vertices) - 1):
        segment = np.linspace(vertices[i], vertices[i+1], num_points)
        points.extend(segment)
    
    return np.array(points)

def star_trajectory(center, size, height, tf, dt=0.01):
    """Generate trajectory for drawing a star using robotics toolbox"""
    # Create robot
    robot = create_puma560()
    
    # Generate star points
    points = generate_star_points(center, size, height)
    
    # Convert points to SE3 poses (end effector pointing down)
    via_points = []
    for p in points:
        T = SE3(p) * SE3.Ry(np.pi)  # End effector pointing down
        via_points.append(T)
    
    # Generate time vector
    t = np.arange(0, tf, dt)
    
    # Create Cartesian trajectory through points using RMRC
    traj = rtb.ctraj(via_points, t)
    
    # Extract joint angles, velocities, and accelerations
    q = np.degrees(traj.q)
    qd = np.degrees(traj.qd)
    qdd = np.zeros_like(q)  # Accelerations not provided by toolbox
    
    # Calculate accelerations using finite differences
    dt = t[1] - t[0]
    qdd[1:-1] = (qd[2:] - qd[:-2]) / (2 * dt)
    qdd[0] = (qd[1] - qd[0]) / dt
    qdd[-1] = (qd[-1] - qd[-2]) / dt
    
    return q, qd, qdd, t