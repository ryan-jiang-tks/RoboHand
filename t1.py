"""
chinese_3d_engraving.py
功能：将汉字转换为三维SE(3)轨迹，支持多格式导出
依赖：numpy, scipy, freetype-py, matplotlib, rosbag (可选)
"""

import json
import freetype
import numpy as np
from scipy.spatial.transform import Rotation
from typing import List, Tuple, Optional

class Chinese3DEngine:
    def __init__(self, font_path: str = "simhei.ttf"):
        """
        初始化字体引擎
        :param font_path: 中文字体文件路径 (.ttf/.otf)
        """
        self.face = freetype.Face(font_path)
        self.face.set_char_size(48 * 64)  # 48pt设计尺寸

    # --------------------------
    # 核心轮廓处理
    # --------------------------
    def get_char_contours(self, char: str) -> Tuple[np.ndarray, np.ndarray, List[Tuple[int, int]]]:
        """提取单字轮廓数据"""
        self.face.load_char(char, freetype.FT_LOAD_NO_SCALE | freetype.FT_LOAD_NO_BITMAP)
        outline = self.face.glyph.outline
        
        points = np.array(outline.points, dtype=np.float32)
        tags = outline.tags
        contours = []
        
        start = 0
        for end in outline.contours:
            contours.append((start, end + 1))
            start = end + 1
            
        return points, tags, contours

    def reconstruct_contour(self, points: np.ndarray, tags: np.ndarray, contour_indices: range) -> np.ndarray:
        """重建闭合轮廓"""
        path = []
        n = len(contour_indices)
        
        for i in range(n):
            idx = contour_indices[i]
            next_idx = contour_indices[(i+1) % n]
            
            if tags[idx] == 1:  # FT_Curve_Tag_On
                if tags[next_idx] == 1:  # 直线段
                    path.append(points[idx])
                else:  # 二次贝塞尔曲线
                    control = points[next_idx]
                    end_idx = contour_indices[(i+2) % n]
                    curve = self._quadratic_bezier(points[idx], control, points[end_idx])
                    path.extend(curve)
                    
        return np.array(path)

    def _quadratic_bezier(self, p0: np.ndarray, p1: np.ndarray, p2: np.ndarray, num: int = 20) -> np.ndarray:
        """二次贝塞尔曲线插值"""
        t = np.linspace(0, 1, num)
        curve = np.outer((1-t)**2, p0) + np.outer(2*(1-t)*t, p1) + np.outer(t**2, p2)
        return curve

    # --------------------------
    # 三维轨迹生成
    # --------------------------
    def generate_3d_poses(self, 
                         text: str,
                         z_height: float = 10.0,
                         layer_step: float = 2.0,
                         spacing: Tuple[float, float] = (30.0, 30.0),
                         base_plane: np.ndarray = np.eye(4)) -> List[np.ndarray]:
        """
        生成多汉字三维SE(3)轨迹
        :param text: 输入文本
        :param z_height: 雕刻总高度 (mm)
        :param layer_step: 分层厚度 (mm)
        :param spacing: (字间距, 行间距) (mm)
        :param base_plane: 基准坐标系变换矩阵 (4,4)
        :return: SE(3)位姿列表 [ (N,4,4) ]
        """
        all_poses = []
        x_offset, y_offset = 0.0, 0.0
        
        for char in text:
            # 获取2D轮廓
            points, tags, contours = self.get_char_contours(char)
            if not contours:
                continue
                
            # 处理每个轮廓
            char_poses = []
            for start, end in contours:
                contour = self.reconstruct_contour(points, tags, range(start, end))
                
                # 三维分层
                layers = int(z_height / layer_step)
                for z in np.linspace(0, z_height, layers):
                    for x, y in contour:
                        T = np.eye(4)
                        T[:3, :3] = Rotation.from_euler('x', 180, degrees=True).as_matrix()
                        T[:3, 3] = [x + x_offset, y + y_offset, z]
                        T = base_plane @ T  # 变换到工作平面
                        char_poses.append(T)
            
            all_poses.extend(char_poses)
            
            # 更新偏移量
            x_offset += spacing[0]
            if (len(all_poses) % 5) == 0:  # 简单换行逻辑
                x_offset = 0
                y_offset -= spacing[1]
                
        return all_poses

    # --------------------------
    # 文件导出
    # --------------------------
    def export_json(self, poses: List[np.ndarray], filename: str):
        """导出为JSON格式"""
        data = {
            "metadata": {
                "units": "mm",
                "format": "SE(3)",
                "num_poses": len(poses)
            },
            "poses": [p.tolist() for p in poses]
        }
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)

    def export_csv(self, poses: List[np.ndarray], filename: str):
        """导出为CSV格式"""
        header = "X,Y,Z,R11,R12,R13,R21,R22,R23,R31,R32,R33\n"
        with open(filename, 'w') as f:
            f.write(header)
            for pose in poses:
                x, y, z = pose[:3, 3]
                R = pose[:3, :3].flatten()
                line = f"{x:.3f},{y:.3f},{z:.3f}," + ",".join(f"{r:.6f}" for r in R)
                f.write(line + "\n")

    def export_kuka_krl(self, poses: List[np.ndarray], filename: str):
        """生成KUKA机器人程序"""
        krl_template = """DEF CHINESE_ENGRAVING()
; Tool/BASE Setup
$TOOL=TOOL_DATA[1]
$BASE=BASE_DATA[0]
$VEL.CP=0.05

; Trajectory Points
{points}

; Return Home
PTP HOME
END
"""
        points_code = []
        for i, pose in enumerate(poses):
            x, y, z = pose[:3, 3]
            points_code.append(f"P{{X {x:.1f}, Y {y:.1f}, Z {z:.1f}, A 0, B 0, C 0}} ; Pt{i+1}")
        
        with open(filename, 'w') as f:
            f.write(krl_template.format(points="\n".join(points_code)))

    # --------------------------
    # 可视化
    # --------------------------
    def plot_3d_trajectory(self, poses: List[np.ndarray]):
        """三维轨迹可视化"""
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        x = [p[0,3] for p in poses]
        y = [p[1,3] for p in poses]
        z = [p[2,3] for p in poses]
        
        ax.plot(x, y, z, 'b-', lw=1)
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        plt.title("3D Engraving Trajectory")
        plt.show()

# --------------------------
# 主程序
# --------------------------
if __name__ == "__main__":
    import argparse

    # 命令行参数解析
    parser = argparse.ArgumentParser(description='汉字三维轨迹生成器')
    parser.add_argument('text', type=str, help='要处理的汉字文本')
    parser.add_argument('--font', type=str, default="simhei.ttf", help='字体文件路径')
    parser.add_argument('--height', type=float, default=10.0, help='雕刻高度(mm)')
    parser.add_argument('--layer', type=float, default=2.0, help='分层厚度(mm)')
    parser.add_argument('--output', type=str, required=True, help='输出文件前缀')
    args = parser.parse_args()

    # 初始化引擎
    engine = Chinese3DEngine(args.font)

    # 生成三维轨迹
    poses_3d = engine.generate_3d_poses(
        text=args.text,
        z_height=args.height,
        layer_step=args.layer
    )

    # 导出文件
    engine.export_json(poses_3d, f"{args.output}.json")
    engine.export_csv(poses_3d, f"{args.output}.csv")
    engine.export_kuka_krl(poses_3d, f"{args.output}.src")
    
    # 可视化
    engine.plot_3d_trajectory(poses_3d)
    
    print(f"处理完成！已生成以下文件：")
    print(f"- {args.output}.json : 标准SE(3)轨迹")
    print(f"- {args.output}.csv  : 通用CSV轨迹")
    print(f"- {args.output}.src  : KUKA机器人程序")