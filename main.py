# from kinematics.forward_kinematics import forward_kinematics
# from kinematics.inverse_kinematics import inverse_kinematics
from trajectory.star_trajectory import star_trajectory
from trajectory.chinese_trajectory import generate_chinese_trajectory
from utils.visualization import plot_trajectory
from utils.data_logging import save_trajectory_to_csv
import argparse
import numpy as np
import sys

def parse_args():
    parser = argparse.ArgumentParser(description='Generate robot trajectories')
    parser.add_argument('type', choices=['star', 'chinese'], help='Type of trajectory to generate')
    parser.add_argument('--text', default='机器人', help='Text for Chinese trajectory')
    parser.add_argument("--spacing", type=float, default=1.1, help="字符间距（米）")  
    parser.add_argument('--center', nargs=3, type=float, default=[0.3, 0.2, 0.5], help='Center position [x y z]')
    parser.add_argument('--size', type=float, default=0.1, help='Pattern size in meters')
    parser.add_argument("--q0", nargs=6, type=float, default=[0,0,0,0,0,0],help="初始关节角（度）")
    parser.add_argument('--height', type=float, default=0.3, help='Writing height in meters')
    parser.add_argument('--time', type=float, default=10.0, help='Total trajectory time')
    parser.add_argument("--disable_parallel", action="store_true",
                    help="禁用并行计算")
    return parser.parse_args()

def main():
    sys.argv = [
        # "chinese_trajectory.py",  # 通常是脚本文件名，可以写成任意内容
        "type", "chinese",  # 执行任务参数，必须
        '--text',"1028JZH",                # <text>：位置参数，必须
        "--size", "0.1",          # 可选参数
        "--spacing", "1",  # 字符间距
        "--height", "0.4",
        "--time", "20",
        "--q0", "10", "20", "10", "30", "40", "50",  # 初始关节角度（共6个）
        "--disable_parallel"# 这是 flag 类型，出现即为 True
    ]
    args = parse_args()
    
    # 初始关节角转换（度→弧度）
    q0_rad = np.radians(args.q0)
    
   
    if args.type == 'star':
        q, qd, qdd, t = star_trajectory(
            center=args.center,
            size=args.size,
            height=args.height,
            tf=args.time
        )
    else:  # chinese
        q, qd, qdd, t = generate_chinese_trajectory(
            text=args.text,
            center=args.center,
            size=args.size,
            height=args.height,
            tf=args.time,
            q0=q0_rad,
            enable_parallel=not args.disable_parallel
        )
    
    # Save and visualize results
    csv_file = save_trajectory_to_csv(q, qd, qdd, t)
    print(f"Trajectory data saved to: {csv_file}")
    plot_trajectory(q, t)

if __name__ == "__main__":
    main()