#!/usr/bin/env python3
"""_summary_
"""

import open3d as o3d

class Visualizer():
    """_summary_
    """
    def __init__(self, visualize_rosbag_data = False):
        self.visualize_rosbag_data = visualize_rosbag_data

    def visualize_pointcloud(points):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        
        intensity = points[:, 3]
        intensity_normalized = (intensity - np.min(intensity)) / (np.max(intensity) - np.min(intensity))
        
        colors = np.tile(intensity_normalized[:, None], (1, 3))
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        o3d.visualization.draw_geometries([pcd])

    def visualize_depth_image(image):
        plt.imshow(image,cmap='gray')
        plt.title('Depth Image')
        plt.axis('off')
        plt.show()

    def visualize_rgb_image(image):
        plt.imshow(image,cmap='rgb')
        plt.title('Depth Image')
        plt.axis('off')
        plt.show()