import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from utils.dh_params import PUMA560_DH_PARAMS as dh
import matplotlib.pyplot as plt
def create_puma560():
    """Create PUMA560 robot model using robotics toolbox"""
    # Convert our DH params to robotics toolbox format
    robot = rtb.DHRobot([
        rtb.RevoluteDH(d=dh[0,1], a=dh[0,2], alpha=dh[0,3]),
        rtb.RevoluteDH(d=dh[1,1], a=dh[1,2], alpha=dh[1,3]),
        rtb.RevoluteDH(d=dh[2,1], a=dh[2,2], alpha=dh[2,3]),
        rtb.RevoluteDH(d=dh[3,1], a=dh[3,2], alpha=dh[3,3]),
        rtb.RevoluteDH(d=dh[4,1], a=dh[4,2], alpha=dh[4,3]),
        rtb.RevoluteDH(d=dh[5,1], a=dh[5,2], alpha=dh[5,3])
    ], name="PUMA560")
    return robot

def test_trajectory():
    """生成并验证轨迹"""
    robot = create_puma560()
    
    # 定义轨迹起止点
    q_start = np.array([0, 0, 0, 0, 0, 0])
    T_target = SE3(0.4, 0.2, 0.5) * SE3.Rx(np.pi/2)
    
    # 逆运动学解算
    sol = robot.ikine_LM(T_target, q0=q_start, tol=1e-6, ilimit=200)
    if not sol.success:
        raise ValueError(f"逆运动学失败：{sol.reason}")
    q_end = sol.q

    # 生成轨迹
    trajectory =rtb.jtraj(q_start, q_end, t=50)
    
    # 轨迹验证
    max_pos_err = max(np.linalg.norm(robot.fkine(q).t - T_target.t) for q in trajectory.q)
    print(f"最大位置误差：{max_pos_err:.2e} m")
    
    # 3D可视化
    robot.plot(trajectory.q,  dt=0.1, vellipse=True)
    

if __name__ == "__main__":
    traj = test_trajectory()
