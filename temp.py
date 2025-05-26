"""
中文汉字3D采样点生成工具
功能：从字体文件生成可雕刻的三维点云轨迹
"""
import freetype
import numpy as np
import open3d as o3d
import pandas as pd
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Chinese3DPathGenerator:
    def __init__(self, font_path="simhei.ttf"):
        self.font_path ="type/GuFengLiShu-2.ttf"

    def get_glyph_points(self, char):
        """提取汉字轮廓点"""
        face = freetype.Face(self.font_path)
        face.set_char_size(48*64)
        face.load_char(char, freetype.FT_LOAD_DEFAULT | freetype.FT_LOAD_NO_BITMAP)
        outline = face.glyph.outline
        return np.array(outline.points, dtype=np.float32), outline.tags

    def extrude_to_3d(self, points_2d, height=10, layers=10):
        """Z轴挤出三维化"""
        z_values = np.linspace(0, height, layers)
        points_3d = np.vstack([
            np.hstack([points_2d, np.full((len(points_2d),1), z)]) 
            for z in z_values
        ])
        return points_3d

    def poisson_sample(self, points, density=0.5):
        """泊松圆盘采样"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        down_pcd = pcd.uniform_down_sample(every_k_points=int(1/density))
        return np.asarray(down_pcd.points)

    def save_to_csv(self, points, filename):
        """保存为CSV"""
        df = pd.DataFrame(points, columns=['X(mm)', 'Y(mm)', 'Z(mm)'])
        df.to_csv(filename, index=False)

    def visualize_2d_outline(self, points_2d):
        """显示2D轮廓"""
        plt.figure(figsize=(10, 10))
        plt.plot(points_2d[:, 0], points_2d[:, 1], 'b-')
        plt.scatter(points_2d[:, 0], points_2d[:, 1], c='r', s=20)
        plt.axis('equal')
        plt.title('2D Character Outline')
        plt.grid(True)
        plt.show()
    
    def visualize_3d_path(self, points_3d):
        """显示3D轨迹"""
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制点云
        ax.scatter(points_3d[:, 0], points_3d[:, 1], points_3d[:, 2], 
                  c=points_3d[:, 2], cmap='viridis', marker='.')
        
        # 绘制轨迹连线
        for i in range(len(points_3d)-1):
            ax.plot([points_3d[i,0], points_3d[i+1,0]],
                   [points_3d[i,1], points_3d[i+1,1]],
                   [points_3d[i,2], points_3d[i+1,2]], 'b-', alpha=0.3)
        
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title('3D Character Path')
        plt.show()

if __name__ == "__main__":
    # 示例：生成"中"字的3D轨迹
    generator = Chinese3DPathGenerator()
    
    # 1. 获取轮廓
    points_2d, _ = generator.get_glyph_points("江")
    generator.visualize_2d_outline(points_2d)
    
    # 2. 三维化处理
    points_3d = generator.extrude_to_3d(points_2d, height=15, layers=8)
    
    # 3. 点云采样优化
    sampled_points = generator.poisson_sample(points_3d, density=0.4)
    generator.visualize_3d_path(points_3d)
    
    # 4. 保存结果
    generator.save_to_csv(sampled_points, "chinese_3d_points.csv")

    print("轨迹文件已生成：chinese_3d_points.csv")