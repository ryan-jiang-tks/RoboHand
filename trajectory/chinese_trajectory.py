"""
chinese_trajectory_optimized.py - 并行化中文书写轨迹生成器

功能：
1. 基于FreeType生成汉字笔画轨迹
2. 使用机器人工具箱进行逆运动学求解
3. 多进程并行加速计算
4. 轨迹验证与数据保存

依赖：
- Python 3.8+
- roboticstoolbox >= 1.0
- numpy
- tqdm
- opencv-python (仅用于可视化)

作者：江知昊
日期：2024-01-20
"""

import sys
import os
import time
import argparse
from multiprocessing import Pool, cpu_count, set_start_method
from functools import partial
from math import radians
from typing import Tuple, List
from spatialmath import SE3
import numpy as np
from tqdm import tqdm
import roboticstoolbox as rtb
from roboticstoolbox.backends.PyPlot import PyPlot

# 项目路径设置
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PROJECT_ROOT)

# 本地模块导入
from utils.robot_model import create_robot
from points.chinese_freetype import ChineseCharacterGenerator
from utils.visualization import plot_trajectory
from trajectory.validation_trajectory import validate_trajectory
from utils.data_logging import save_trajectory_to_csv

# 多进程初始化（Windows必须设置）
set_start_method("spawn", force=True)

# --------------------------
# 核心函数定义
# --------------------------

def compute_ik(args: Tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    """
    多进程计算逆运动学的独立函数
    参数：
        args: (位姿矩阵4x4, 初始关节角猜测)
    返回：
        成功：关节角数组(6,)
        失败：None
    """
    pose_array, q0_initial = args
    try:
        # 每个进程独立创建机器人实例
        robot = create_robot()
        T = SE3(pose_array)
        sol = robot.ikine_LM(
            T, 
            q0=q0_initial,
            ilimit=200,  # 减少迭代次数加速计算
            tol=1e-4      # 适当放宽收敛容差
        )
        return sol.q if sol.success else None
    except Exception as e:
        print(f"\nIK计算错误：{str(e)}")
        return None

def parallel_ik_solver(
    all_poses: List[SE3],
    q0_initial: np.ndarray,
    chunk_size: int = 100,
    n_workers: int = None
) -> Tuple[np.ndarray, int]:
    """
    并行化逆运动学求解器
    参数：
        all_poses: 所有目标位姿列表
        q0_initial: 初始关节角猜测
        chunk_size: 任务分块大小
        n_workers: 使用的进程数
    返回：
        q_trajectory: 关节角轨迹
        failure_count: IK失败次数
    """
    # 参数预处理
    pose_arrays = [T.A for T in all_poses]  # 转换为可序列化的numpy数组
    total_points = len(pose_arrays)
    n_workers = n_workers or max(1, cpu_count() - 1)
    
    q_results = []
    failure_count = 0
    current_q = q0_initial.copy()
    
    # 进度条配置
    pbar = tqdm(
        total=total_points,
        desc="并行IK求解",
        unit="point",
        dynamic_ncols=True
    )

    with Pool(processes=n_workers) as pool:
        for i in range(0, total_points, chunk_size):
            # 分块处理
            chunk_end = min(i + chunk_size, total_points)
            chunk_poses = pose_arrays[i:chunk_end]
            
            # 生成任务参数
            task_args = [(pose, current_q) for pose in chunk_poses]
            
            # 并行计算
            chunk_results = pool.map(compute_ik, task_args)
            
            # 处理结果
            for q in chunk_results:
                if q is not None:
                    q_results.append(q)
                    current_q = q  # 更新初始猜测
                else:
                    q_results.append(current_q)  # 沿用上次有效值
                    failure_count += 1
            
            # 更新进度
            pbar.update(len(chunk_results))
    
    pbar.close()
    return np.array(q_results), failure_count

def generate_chinese_trajectory(
    text: str,
    center: List[float] = [0.3, 0.2, 0.5],
    size: float = 0.2,
    height: float = 0.5,
    tf: float = 10.0,
    dt: float = 0.01,
    q0: np.ndarray = None,
    enable_parallel: bool = True
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    主轨迹生成函数
    参数：
        text: 要书写的汉字
        center: 书写中心坐标[x,y,z]
        size: 字符尺寸（米）
        height: 抬笔高度（米）
        tf: 总时间（秒）
        dt: 时间步长（秒）
        q0: 初始关节角（弧度）
        enable_parallel: 启用并行计算
    返回：
        q: 关节位置轨迹
        qd: 关节速度轨迹
        qdd: 关节加速度轨迹
        t: 时间向量
    """
    # 初始化检查
    if not text:
        raise ValueError("输入文本不能为空")
    if q0 is None:
        q0 = np.zeros(6)
    elif q0.shape != (6,):
        raise ValueError("初始关节角必须为6维数组")

    # 生成字符位姿
    char_gen = ChineseCharacterGenerator()
    poses = char_gen.generate_se3_trajectory(
        text=text,
        center=center,
        size=size,
        height=height
    )
    total_segments = len(poses) - 1
    samples = int(tf / dt)
    pts_per_segment = max(2,(samples // len(poses)))

    # 生成笛卡尔轨迹
    all_poses = []
    for i in range(total_segments):
        traj = rtb.ctraj(poses[i], poses[i+1], pts_per_segment)
        all_poses.extend(traj)

    # 逆运动学求解
    start_time = time.time()
    if enable_parallel:
        q_traj, failures = parallel_ik_solver(
            all_poses=all_poses,
            q0_initial=q0,
            chunk_size=100
        )
    else:
        # 单进程备用方案
        q_traj, failures = [], 0
        current_q = q0.copy()
        for T in tqdm(all_poses, desc="单进程IK求解"):
            sol = create_robot().ikine_LM(T, q0=current_q)
            if sol.success:
                q_traj.append(sol.q)
                current_q = sol.q
            else:
                q_traj.append(current_q)
                failures += 1
        q_traj = np.array(q_traj)
    
    # 后处理
    t = np.linspace(0, tf, len(q_traj))
    dt_actual = t[1] - t[0]
    
    # 计算导数
    qd = np.gradient(q_traj, dt_actual, axis=0)
    qdd = np.gradient(qd, dt_actual, axis=0)
    
    # 验证与保存
    all_q_smooth, all_qd_smooth, all_qdd_smooth,t_new = validate_trajectory(
        q_traj, qd, qdd, t, poses
    )
    
    csv_path = save_trajectory_to_csv(
        q=np.degrees(all_q_smooth),
        qd=np.degrees(all_qd_smooth),
        qdd=np.degrees(all_qdd_smooth),
        t=t_new,
        directory=os.path.join("data", "chinese_generated"),
        prefix=f"chinese_{text}_"
    )

    print(f"\n轨迹生成完成，耗时：{time.time()-start_time:.2f}s")
    print(f"失败点：{failures}/{len(q_traj)} ({failures/len(q_traj)*100:.2f}%)")
    print(f"数据已保存至：{csv_path}")

    # # Validate and adjust trajectory
    # vel_spikes, acc_spikes = validate_trajectory(q_traj, qd, qdd, t, poses)
    
    # # Add more points around problematic areas
    # if len(vel_spikes[0]) > 0 or len(acc_spikes[0]) > 0:
    #     print("\nAdjusting trajectory timing for smoothness...")
    #     problem_indices = np.unique(np.concatenate([vel_spikes[0], acc_spikes[0]]))
        
    #     # Adjust time allocation around problem areas
    #     t_new = np.copy(t)
    #     for idx in problem_indices:
    #         # Extend time around problematic points
    #         if idx > 0 and idx < len(t)-1:
    #             t_new[idx:] += dt * 2
                
    #     # Recalculate velocities and accelerations with new timing
    #     t = t_new
    #     qd = np.gradient(q_traj, t, axis=0)
    #     qdd = np.gradient(qd, t, axis=0)
        
    #     # Validate again
    #     validate_trajectory(q_traj, qd, qdd, t, poses)
    
    # Validate and smooth trajectory if needed
    all_q_smooth, all_qd_smooth, all_qdd_smooth,t_new = validate_trajectory(
        all_q_smooth, all_qd_smooth, all_qdd_smooth, t_new, poses
    )
    
    return all_q_smooth, all_qd_smooth, all_qdd_smooth, t_new

# --------------------------
# 命令行接口
# --------------------------

def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description="中文书写轨迹生成器")
    parser.add_argument("text", type=str, help="要书写的汉字")
    parser.add_argument("--size", type=float, default=0.2, help="字符尺寸（米）")
    parser.add_argument("--height", type=float, default=0.3, help="抬笔高度（米）")
    parser.add_argument("--time", type=float, default=40.0, help="总时间（秒）")
    parser.add_argument("--q0", nargs=6, type=float, default=[0,0,0,0,0,0],
                        help="初始关节角（度）")
    parser.add_argument("--disable_parallel", action="store_true",
                        help="禁用并行计算")
    return parser.parse_args()

# --------------------------
# 主程序
# --------------------------

if __name__ == "__main__":
    sys.argv = [
    "chinese_trajectory.py",  # 通常是脚本文件名，可以写成任意内容
    "昊",                # <text>：位置参数，必须
    "--size", "0.1",          # 可选参数
    "--height", "0.4",
    "--time", "40",
    "--q0", "10", "20", "10", "30", "40", "50",  # 初始关节角度（共6个）
    # 这是 flag 类型，出现即为 True
]
    args = parse_arguments()
  
    # 初始关节角转换（度→弧度）
    q0_rad = np.radians(args.q0)
    
    # 生成轨迹
    q, qd, qdd, t = generate_chinese_trajectory(
        text=args.text,
        size=args.size,
        height=args.height,
        tf=args.time,
        q0=q0_rad,
        enable_parallel=not args.disable_parallel
    )
    
    # 可视化
    plot_trajectory(np.degrees(q), t)