import csv
import os
from datetime import datetime
import numpy as np

def save_trajectory_to_csv(q, qd, qdd, t, directory="data", prefix="trajectory_"):
    """Save trajectory data to CSV file
    Args:
        q, qd, qdd: Joint trajectories
        t: Time vector
        directory: Directory to save file
        prefix: Prefix for filename
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs(directory, exist_ok=True)
    filename = os.path.join(directory, f"{prefix}{timestamp}.csv")
    
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # # Write header
        # writer.writerow(['Time'] + 
        #                [f'q{i+1}' for i in range(6)] +
        #                [f'qd{i+1}' for i in range(6)] +
        #                [f'qdd{i+1}' for i in range(6)])
        
        # Write data
        for i in range(len(t)):
            row = [t[i]]
            for j in range(6):
                row.extend([q[i,j], qd[i,j], qdd[i,j]])
            writer.writerow(row)
            
    return filename

def save_q(q_data, name, directory="data/joints"):
    """Save joint angles data to CSV
    Args:
        q_data: Joint angles in degrees (N x 6 array)
        name: Name prefix for the file
        directory: Directory to save the file
    Returns:
        filename: Path to saved file
    """
    os.makedirs(directory, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(directory, f'q_{name}_{timestamp}.csv')
    np.savetxt(filename, q_data, delimiter=',', fmt='%.6f')
    print(f"Joint angles saved to: {filename}")
    return filename