import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb
from utils.visualization import plot_star_points

def generate_star_points(center, size, height, num_points=10):
    """Generate star points as SE3 poses"""
    angles = np.linspace(0, 2*np.pi, 11)[:-1] - np.pi/2
    inner_size = size * 0.382
    
    vertices = []
    for i in range(10):
        r = size if i % 2 == 0 else inner_size
        x = center[0] + r * np.cos(angles[i])
        y = center[1] + r * np.sin(angles[i])
        z = height
        
        Rz = SE3.Rz(angles[i])
        Ry = SE3.Ry(np.pi)
        T = SE3(x, y, z) * Rz * Ry
        vertices.append(T)
    
    vertices.append(vertices[0])
    
    points = []
    for i in range(len(vertices)-1):
        ctraj = rtb.ctraj(vertices[i], vertices[i+1], num_points)
        points.extend(ctraj)
    
    return points
