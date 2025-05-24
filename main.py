# from kinematics.forward_kinematics import forward_kinematics
# from kinematics.inverse_kinematics import inverse_kinematics
from trajectory.star_trajectory import star_trajectory
from utils.visualization import plot_trajectory
from utils.data_logging import save_trajectory_to_csv
# from utils.transformations import dh_matrix, get_R0_3

def main():
    # Define star parameters
    center = [0.4, 0.2, 0.5]
    size = 0.2
    height = 0.6
    
    # Generate star trajectory
    q_traj_star, qd_traj_star, qdd_traj_star, t_star = star_trajectory(center, size, height, 10)
    
    # Save and visualize results
    csv_file_star = save_trajectory_to_csv(q_traj_star, qd_traj_star, qdd_traj_star, t_star)
    print(f"Star trajectory data saved to: {csv_file_star}")
    plot_trajectory(q_traj_star, t_star)

if __name__ == "__main__":
    main()
