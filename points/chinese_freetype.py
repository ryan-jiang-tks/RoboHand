import freetype
import numpy as np
from spatialmath import SE3, SO3, base
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import pandas as pd
from datetime import datetime

class ChineseCharacterGenerator:
    def __init__(self, font_path="type/SourceHanSansCN-Normal.otf"):
        self.font_path = font_path
        self.face = freetype.Face(font_path)
        # 提高字体渲染精度
        self.face.set_char_size(width=0, height=40*40, hres=40, vres=40)
        # 设置numpy全局精度
        np.set_printoptions(precision=15)
    
    def extract_char_contours(self, char):
        """提取单个汉字的高精度轮廓数据"""  
        # 设置更高质量的加载选项
        self.face.load_char(char, 
                          freetype.FT_LOAD_NO_BITMAP | 
                          freetype.FT_LOAD_NO_HINTING | 
                          freetype.FT_LOAD_NO_AUTOHINT |
                          freetype.FT_LOAD_NO_SCALE)
        
        outline = self.face.glyph.outline
        
        # 使用双精度存储所有点
        all_points = np.array(outline.points, dtype=np.float64)
        
        # 归一化使用双精度计算
        min_xy = np.min(all_points, axis=0)
        max_xy = np.max(all_points, axis=0)
        scale = np.max(max_xy - min_xy)

        # 提取并细化轮廓
        contours = []
        start = 0
        for end in outline.contours:
            contour = all_points[start:end + 1]
            # 对轮廓进行插值细化
            refined_contour = self.interpolate_curve(contour, num_points=5)
            # 使用统一的归一化参数
            normalized_contour = (refined_contour - min_xy) / scale
            contours.append(normalized_contour)
            start = end + 1
        
        return contours
    
    def interpolate_curve(self, points, num_points=20):
        """使用双精度贝塞尔曲线插值"""
        t = np.linspace(0, 1, num_points, dtype=np.float64)
        interpolated = []
        
        for i in range(len(points) - 1):
            # 线性插值
            if i + 1 < len(points):
                p0 = points[i]
                p1 = points[i+1]
                for ti in t:
                    point = (1-ti) * p0 + ti * p1
                    interpolated.append(point)
        
        return np.array(interpolated, dtype=np.float64)
    
    def interpolate_between_poses(self, pose1, pose2, num_points=5):
        """在两个位姿之间插值"""
        interpolated = []
        # 跳过相同的位姿
        if np.allclose(pose1.t, pose2.t, atol=1e-6):
            return interpolated
        
        # 在两个位姿之间进行线性插值
        for t in np.linspace(0, 1, num_points+2)[1:-1]:
            # 位置插值
            pos = (1-t) * pose1.t + t * pose2.t
            
            # 方向插值
            direction = pose2.t - pose1.t
            direction_xy = np.array([direction[0], direction[1], 0])
            if np.linalg.norm(direction_xy) > 1e-6:
                direction_xy = direction_xy / np.linalg.norm(direction_xy)
                z_axis = np.array([0, 0, -1])
                x_axis = direction_xy
                y_axis = np.cross(z_axis, x_axis)
                # R = SO3(np.array([x_axis, y_axis, z_axis]/)T)
                R = SO3(np.eye(3))
                interpolated.append(SE3.Rt(R, pos))
            
        return interpolated

    def generate_se3_trajectory(self, text, center=[-0.5, 0.2, 0.3], size=0.07, 
                              height=0.02, char_spacing=1.1):
        """生成多个汉字的SE3轨迹点
        Args:
            text: 要生成的汉字字符串
            center: 起始位置 [x,y,z]
            size: 字体大小(米)
            height: 抬笔高度(米)
            char_spacing: 字间距系数
        Returns:
            List[SE3]: SE3位姿序列
        """
        poses = []
        center = np.array(center, dtype=np.float64)
        POINT_THRESHOLD = 1e-6  # 重合点判断阈值

        raw_poses = []  # Store initial poses
        # Generate initial poses
        for i, char in enumerate(text):
            # Move along X axis, keeping Y fixed, varying Z for height
            char_pos = center + np.array([i * size * char_spacing, 0, 0])
            contours = self.extract_char_contours(char)
            coutour_start=[]
            pos = np.array([char_pos[0], char_pos[1], char_pos[2]])
            R = SO3(np.array([
                    [1, 0, 0],  # X axis points forward
                    [0, 1, 0],  # Y axis points left
                    [0, 0, 1]   # Z axis points up
                ]))
            poses.append(SE3.Rt(R, pos))
        
            for contour in contours:
                # # Add pen-up motion if needed
                # if len(poses) > 0:
                #     start_pos = char_pos + np.array([contour[0][0]*size, 0, contour[0][1]*size])  # Swap Y->Z
                #     poses.append(SE3(start_pos))
                
                for j in range(len(contour)):
                    point = contour[j]
                    # Map Y coordinate to Z axis
                    pos = char_pos + np.array([point[0]*size, 0, point[1]*size])  # Swap Y->Z
                    
                    # Create fixed rotation matrix for XOZ plane writing
                    R = SO3(np.array([
                        [1, 0, 0],  # X axis points forward
                        [0, 1, 0],  # Y axis points left
                        [0, 0, 1]   # Z axis points up
                    ]))
                    if(j==1):
                        coutour_start=SE3.Rt(R, pos)
                    poses.append(SE3.Rt(R, pos))
                poses.append(coutour_start)
            #go back to init point
            pos = np.array([char_pos[0], char_pos[1], char_pos[2]])
            R = SO3(np.array([
                    [1, 0, 0],  # X axis points forward
                    [0, 1, 0],  # Y axis points left
                    [0, 0, 1]   # Z axis points up
                ]))
            poses.append(SE3.Rt(R, pos))
        
        
        # Add interpolated poses
        interpolated_poses = []
        for i in range(len(poses)-1):
            interpolated_poses.append(poses[i])
            interpolated = self.interpolate_between_poses(poses[i], poses[i+1])
            interpolated_poses.extend(interpolated)
        interpolated_poses.append(poses[-1])
        
        # 保存生成的位姿
        self.save_poses_to_csv(interpolated_poses, text)
        return interpolated_poses
    
    def save_poses_to_csv(self, poses, text):
        """Save poses as pure 6D Cartesian data (X, Y, Z in mm, Roll, Pitch, Yaw in degrees)"""
        save_dir = os.path.join("data", "points", "chinese")
        os.makedirs(save_dir, exist_ok=True)
        
        # Extract and stack 6D pose data
        data = []
        for pose in poses:
            pos = pose.t * 1000  # Convert position from meters to millimeters
            rpy_rad = base.tr2rpy(pose.A, order='xyz')
            rpy_deg = np.degrees(rpy_rad)  # Convert rotations to degrees
            data.append(np.concatenate([pos, rpy_deg]))
        
        # Convert to numpy array and save without headers
        data_array = np.array(data)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(save_dir, f'chinese_poses_6d_{text}_{timestamp}.csv')
        np.savetxt(filename, data_array, delimiter=',', fmt='%.6f')
        print(f"6D poses saved to: {filename} (rotations in degrees)")
        return filename

    def visualize_trajectory(self, poses):
        """可视化生成的轨迹"""
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 提取位置点
        points = np.array([pose.t for pose in poses])
        
        # 绘制轨迹
        ax.plot(points[:, 0], points[:, 1], points[:, 2], 'b.-')
        
        # # 每隔几个点显示坐标系
        # for i in range(0, len(poses), 10):
        #     pose = poses[i]
        #     origin = pose.t
        #     axes = pose.R * 0.01  # 缩放坐标轴显示
            
        #     # 绘制坐标轴
        #     ax.quiver(origin[0], origin[1], origin[2],
        #              axes[0,0], axes[1,0], axes[2,0], color='r')  # x轴
        #     ax.quiver(origin[0], origin[1], origin[2],
        #              axes[0,1], axes[1,1], axes[2,1], color='g')  # y轴
        #     ax.quiver(origin[0], origin[1], origin[2],
        #              axes[0,2], axes[1,2], axes[2,2], color='b')  # z轴
        ax.set_xlim([0.15,1.2])
        ax.set_ylim([0.15,1.2])
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        plt.show()

if __name__ == "__main__":
    # 测试生成器
    generator = ChineseCharacterGenerator()
    poses = generator.generate_se3_trajectory("1028JZH", 
                                           center=[0, 0.2, 0.3],
                                           size=0.1,
                                           height=0.3)
    generator.visualize_trajectory(poses)
