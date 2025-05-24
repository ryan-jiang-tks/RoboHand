import numpy as np
from utils.transformations import dh_matrix

def forward_kinematics(joints_deg, dh_params):
    T = np.eye(4)
    for i in range(6):
        theta = joints_deg[i] + dh_params[i][0]
        d = dh_params[i][1]
        a = dh_params[i][2]
        alpha = dh_params[i][3]
        Ti = dh_matrix(theta, d, a, alpha)
        T = T @ Ti
    return T
