import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb
from kinematics.forward_kinematics import forward_kinematics
from kinematics.inverse_kinematics import inverse_kinematics, create_robot
from utils.dh_params import PUMA560_DH_PARAMS
import matplotlib.pyplot as plt

def test_cartesian_frames():
    """Test IK->FK pipeline starting from Cartesian frames"""
    # Define test frames with different positions and orientations
    test_frames = [
        # Position,                Orientation
        ([0.4, 0.0, 0.5],        SE3.Ry(np.pi/2)),     # Front frame
        ([0.3, 0.3, 0.5],        SE3.Ry(np.pi/2) * SE3.Rx(np.pi/4)),  # Side frame
        ([0.4, -0.2, 0.4],       SE3.Ry(-np.pi/4) * SE3.Rz(np.pi/4))   # Lower frame
    ]
    
    for pos, orient in test_frames:
        print(f"\nTesting frame at position: {pos}")
        
        # Create target transformation
        T_target = SE3(pos) * orient
        print("Target transform:")
        print(T_target)
        
        try:
            # Inverse kinematics
            q = inverse_kinematics(T_target.A)
            print(f"\nComputed joint angles (deg):\n{q}")
            
            # Forward kinematics verification
            T_achieved = forward_kinematics(q, PUMA560_DH_PARAMS)
            
            # Calculate and display errors
            pos_error = np.linalg.norm(T_target.t - T_achieved[:3, 3]) * 1000  # mm
            rot_error = np.degrees(np.arccos((np.trace(T_target.R @ T_achieved[:3, :3].T) - 1)/2))
            
            print(f"\nPosition error: {pos_error:.3f} mm")
            print(f"Orientation error: {rot_error:.3f} deg")
            
            # Visualize frames
            fig = plt.figure(figsize=(12, 8))
            ax = fig.add_subplot(111, projection='3d')
            
            # Plot target frame
            plot_frame(ax, T_target.A, scale=0.1, label='Target')
            # Plot achieved frame
            plot_frame(ax, T_achieved, scale=0.1, label='Achieved')
            
            # Set plot properties
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.legend()
            plt.show()
            
        except Exception as e:
            print(f"Failed: {str(e)}")

def plot_frame(ax, T, scale=0.1, label=''):
    """Plot coordinate frame"""
    origin = T[:3, 3]
    x_axis = origin + scale * T[:3, 0]
    y_axis = origin + scale * T[:3, 1]
    z_axis = origin + scale * T[:3, 2]
    
    ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]], 'r-', linewidth=2)
    ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]], 'g-', linewidth=2)
    ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]], 'b-', linewidth=2)
    ax.scatter(*origin, c='k', marker='o', s=100, label=label)

def test_cartesian_kinematics():
    """Test IK->FK pipeline with Cartesian poses"""
    # Create test Cartesian poses with different orientations
    test_poses = [
        (np.array([0.4, 0, 0.6]), SE3.Ry(np.pi)),      # Front, facing down
        (np.array([0.3, 0.3, 0.5]), SE3.Ry(3*np.pi/4)), # Side, angled
        (np.array([0.4, -0.2, 0.4]), SE3.Ry(np.pi/2))   # Lower, horizontal
    ]
    
    robot = create_robot()
    configs = ['righty_up_noflip', 'lefty_up_noflip', 'righty_down_noflip']
    
    for pos, orient in test_poses:
        print(f"\nTesting Cartesian pose: pos={pos}, orient={orient}")
        
        # Create target transformation
        T_target = SE3(pos) * orient
        
        for config in configs:
            print(f"\nTesting configuration: {config}")
            try:
                # Inverse kinematics
                q = inverse_kinematics(T_target.A, config=config)
                print(f"Joint angles (deg): {q}")
                
                # Forward kinematics verification
                T_achieved = forward_kinematics(q, PUMA560_DH_PARAMS)
                
                # Calculate errors
                pos_error = np.linalg.norm(T_target.t - T_achieved[:3, 3]) * 1000  # mm
                rot_error = np.degrees(np.arccos((np.trace(T_target.R @ T_achieved[:3, :3].T) - 1)/2))
                
                print(f"Position error: {pos_error:.3f} mm")
                print(f"Orientation error: {rot_error:.3f} deg")
                
                # Visualize if error is large
                if pos_error > 1.0 or rot_error > 1.0:
                    fig = plt.figure(figsize=(10, 8))
                    ax = fig.add_subplot(111, projection='3d')
                    # Plot target and achieved positions
                    ax.scatter([T_target.t[0]], [T_target.t[1]], [T_target.t[2]], 
                             c='b', marker='o', label='Target')
                    ax.scatter([T_achieved[0,3]], [T_achieved[1,3]], [T_achieved[2,3]], 
                             c='r', marker='x', label='Achieved')
                    ax.set_xlabel('X (m)')
                    ax.set_ylabel('Y (m)')
                    ax.set_zlabel('Z (m)')
                    ax.legend()
                    plt.show()
                
            except Exception as e:
                print(f"Failed: {str(e)}")

def plot_workspace_slice():
    """Plot a slice of the robot workspace"""
    robot = create_robot()
    
    # Create grid of points in XY plane
    x = np.linspace(-0.8, 0.8, 50)
    y = np.linspace(-0.8, 0.8, 50)
    X, Y = np.meshgrid(x, y)
    Z = np.zeros_like(X) + 0.5  # Fixed height
    
    # Test reachability
    reachable = np.zeros_like(X, dtype=bool)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            T = SE3([X[i,j], Y[i,j], Z[i,j]])
            sol = robot.ikine_LM(T)
            reachable[i,j] = sol.success
    
    # Plot
    plt.figure(figsize=(10, 8))
    plt.contourf(X, Y, reachable, cmap='RdYlBu')
    plt.colorbar(label='Reachable')
    plt.title('Robot Workspace at Z=0.5m')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    # test_cartesian_frames()
    test_cartesian_kinematics()
    # plot_workspace_slice()  # Keep workspace visualization
