import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import matplotlib.pyplot as plt

# 创建机械臂模型
robot = rtb.models.DH.Puma560()

# 定义轨迹起点和终点
start = SE3(0.6, 0.2, 0.1)
end = SE3(0.5, -0.2, 0.3)

# 生成笛卡尔轨迹（50个点）
traj = rtb.ctraj(start, end, 50)

# 逆运动学求解
q_traj = []
q0 = robot.qz  # 初始关节角

for T in traj:
    sol = robot.ikine_LM(T, q0=q0)
    if sol.success:
        q_traj.append(sol.q)
        q0 = sol.q
    else:
        q_traj.append(q0)

q_traj = np.array(q_traj)

# 绘制关节角度
plt.plot(q_traj)
plt.xlabel('Time Step')
plt.ylabel('Joint Angles (rad)')
plt.show()

# 动画演示
robot.plot(q_traj, dt=0.05, block=True)