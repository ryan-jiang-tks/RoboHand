import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb
from scipy.interpolate import interp1d

class ChineseStroke:
    def __init__(self, points, pen_up=False):
        self.points = points  # Nx2 array of normalized coordinates
        self.pen_up = pen_up  # True if this is a transition stroke

def generate_chinese_points(character_strokes, center, size, height, surface_normal=[0, 0, 1], num_points=20):
    """Generate SE3 poses for writing Chinese characters
    Args:
        character_strokes: List of ChineseStroke objects
        center: [x, y, z] center position of the writing surface
        size: Size of character in meters
        height: Height above surface for pen-up movements
        surface_normal: Normal vector of writing surface
        num_points: Points per stroke
    Returns:
        List of SE3 poses for the complete character
    """
    # Create rotation matrix for surface orientation
    z_axis = np.array(surface_normal) / np.linalg.norm(surface_normal)
    x_axis = np.cross([0, 1, 0], z_axis)  # Default writing direction
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    R_surface = np.vstack([x_axis, y_axis, z_axis]).T
    
    poses = []
    for stroke in character_strokes:
        stroke_points = []
        for i in range(len(stroke.points)-1):
            # Interpolate between stroke points
            t = np.linspace(0, 1, num_points)
            x = np.linspace(stroke.points[i][0], stroke.points[i+1][0], num_points)
            y = np.linspace(stroke.points[i][1], stroke.points[i+1][1], num_points)
            
            # Scale and transform points
            for xi, yi, ti in zip(x, y, t):
                # Position in surface coordinates
                pos = np.array([
                    center[0] + size * (xi - 0.5),
                    center[1] + size * (yi - 0.5),
                    center[2] + (height if stroke.pen_up else 0)
                ])
                
                # Calculate orientation
                if i < len(stroke.points)-2:
                    dx = stroke.points[i+1][0] - stroke.points[i][0]
                    dy = stroke.points[i+1][1] - stroke.points[i][1]
                    angle = np.arctan2(dy, dx)
                    R_stroke = SE3.Rz(angle).R
                else:
                    R_stroke = np.eye(3)
                
                # Combine surface and stroke orientations
                R = R_surface @ R_stroke
                
                # Create SE3 pose
                T = SE3(pos) * SE3(R)
                poses.append(T)
    
    return poses

# Example character strokes (一)
def generate_yi_character():
    """Generate strokes for Chinese character 一 (yī)"""
    horizontal = ChineseStroke(np.array([
        [0.2, 0.5],
        [0.8, 0.5]
    ]))
    return [horizontal]

# Example usage
if __name__ == "__main__":
    # Test with simple horizontal stroke (一)
    center = [0.4, 0.2, 0.5]  # Center position
    size = 0.2  # Character size in meters
    height = 0.05  # Pen-up height
    
    # Generate poses for character
    strokes = generate_yi_character()
    poses = generate_chinese_points(strokes, center, size, height)
    
    # Visualize trajectory
    points = np.array([pose.t for pose in poses])
    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(points[:, 0], points[:, 1], points[:, 2])
    plt.show()
