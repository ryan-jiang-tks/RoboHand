import numpy as np
from math import radians, degrees

def joint_space_trajectory(q_start_deg, q_end_deg, tf, dt=0.01):
        """
        Cubic polynomial trajectory in joint space
        Input angles in degrees
        Returns angles in degrees
        """
        t = np.arange(0, tf, dt)
        
        # Convert to radians for calculation
        q_start = np.radians(q_start_deg)
        q_end = np.radians(q_end_deg)
        
        a0 = q_start
        a1 = np.zeros(6)
        a2 = 3/(tf**2)*(q_end - q_start)
        a3 = -2/(tf**3)*(q_end - q_start)
        
        # Calculate trajectories in radians
        q_rad = a0 + a1*t[:, np.newaxis] + a2*t[:, np.newaxis]**2 + a3*t[:, np.newaxis]**3
        qd_rad = a1 + 2*a2*t[:, np.newaxis] + 3*a3*t[:, np.newaxis]**2
        qdd_rad = 2*a2 + 6*a3*t[:, np.newaxis]
        
        # Convert back to degrees for output
        q = np.degrees(q_rad)
        qd = np.degrees(qd_rad)  # convert velocity to deg/s
        qdd = np.degrees(qdd_rad)  # convert acceleration to deg/s²
        
        return q, qd, qdd, t