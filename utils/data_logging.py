import csv
import os
from datetime import datetime

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