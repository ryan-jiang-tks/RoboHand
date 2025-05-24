import csv
import os
from datetime import datetime

def save_trajectory_to_csv( q, qd, qdd, t):
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