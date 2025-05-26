import freetype
import numpy as np
from spatialmath import SE3,SO3
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class ChineseCharacterGenerator:
    def __init__(self, font_path="type/GuFengLiShu-2.ttf"):
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
                p0 = points[i].astype(np.float64)
                p1 = points[i+1].astype(np.float64)
                for ti in t:
                    point = (1-ti) * p0 + ti * p1
                    interpolated.append(point)
        
        return np.array(interpolated, dtype=np.float64)
    
    def generate_se3_trajectory(self, text, center=[0.3, 0.2, 0.5], size=0.1, 
                              height=0.02, char_spacing=1.7):
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

        for i, char in enumerate(text):
            char_pos = center + np.array([i * size * char_spacing, 0, 0])
            contours = self.extract_char_contours(char)
            
            for contour in contours:
                # 添加抬笔动作
                if len(poses) > 0:
                    # lift_pos = poses[-1].t + np.array([0, 0, height])
                    # poses.append(SE3(lift_pos))
                    start_pos = char_pos + np.array([contour[0][0]*size, contour[0][1]*size, 0])
                    poses.append(SE3(start_pos))
                
                # 生成轨迹点
                last_valid_direction = None
                for j in range(len(contour)):
                    point = contour[j]
                    pos = char_pos + np.array([point[0]*size, point[1]*size, 0])
                    
                    # 计算方向
                    if j < len(contour) - 1:
                        next_point = contour[j+1]
                        direction = next_point - point
                        dist = np.linalg.norm(direction)
                        
                        # 处理重合点情况
                        if dist < POINT_THRESHOLD:
                            # 如果有上一个有效方向，使用它
                            if last_valid_direction is not None:
                                direction = last_valid_direction
                            # 否则尝试查找下一个非重合点
                            else:
                                for k in range(j+2, len(contour)):
                                    future_direction = contour[k] - point
                                    if np.linalg.norm(future_direction) >= POINT_THRESHOLD:
                                        direction = future_direction
                                        break
                                else:
                                    # 如果找不到非重合点，使用默认方向
                                    direction = np.array([1.0, 0.0])
                        else:
                            direction = direction / dist
                            last_valid_direction = direction
                        
                        # 创建旋转矩阵
                        z_axis = np.array([0, 0, -1])
                        x_axis = np.array([direction[0], direction[1], 0])
                        x_axis = x_axis / np.linalg.norm(x_axis)
                        y_axis = np.cross(z_axis, x_axis)
                        R = SO3(np.array([x_axis, y_axis, z_axis]).T)
                    else:
                        # 最后一个点使用最后的有效方向
                        if last_valid_direction is not None:
                            x_axis = np.array([last_valid_direction[0], last_valid_direction[1], 0])
                            z_axis = np.array([0, 0, -1])
                            y_axis = np.cross(z_axis, x_axis)
                            R = SO3(np.array([x_axis, y_axis, z_axis]).T)
                        else:
                            R = SO3(np.eye(3))
                    
                    poses.append(SE3.Rt(R, pos))
                
        return poses
    
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
    poses = generator.generate_se3_trajectory("江知昊", 
                                           center=[0.3, 0.2, 0.5],
                                           size=0.2,
                                           height=0.02)
    generator.visualize_trajectory(poses)
