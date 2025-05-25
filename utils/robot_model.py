import roboticstoolbox as rtb
import numpy as np
from utils.dh_params import PUMA560_DH_PARAMS as dh
def create_robot():
    """Create PUMA560 robot model"""
    robot = rtb.DHRobot([
        rtb.RevoluteDH(d=dh[0,1], a=dh[0,2], alpha=dh[0,3]),
        rtb.RevoluteDH(d=dh[1,1], a=dh[1,2], alpha=dh[1,3]),
        rtb.RevoluteDH(d=dh[2,1], a=dh[2,2], alpha=dh[2,3]),
        rtb.RevoluteDH(d=dh[3,1], a=dh[3,2], alpha=dh[3,3]),
        rtb.RevoluteDH(d=dh[4,1], a=dh[4,2], alpha=dh[4,3]),
        rtb.RevoluteDH(d=dh[5,1], a=dh[5,2], alpha=dh[5,3])
    ], name="PUMA560")
    return robot