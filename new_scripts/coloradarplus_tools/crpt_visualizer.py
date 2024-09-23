#!/usr/bin/env python3
"""_summary_
"""

import os
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

# TODO: these should go elsewhere
def load_bin_file(file_path, shape, dtype=np.float32):
    """
    Load binary file and reshape it into the desired shape.
    """
    return np.fromfile(file_path, dtype=dtype).reshape(shape)

def transform_point_cloud(point_cloud, pose_matrix):
    """
    Apply a 4x4 transformation matrix to a point cloud.
    """
    # Add homogeneous coordinate to the point cloud
    ones = np.ones((point_cloud.shape[0], 1))
    homogeneous_points = np.hstack([point_cloud, ones])

    # Apply transformation
    transformed_points = homogeneous_points.dot(pose_matrix.T)[:, :3]
    return transformed_points

class Visualizer():
    """_summary_
    """
    def __init__(self):
        pass

    def visualize_pointcloud(self, points):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        
        intensity = points[:, 3]
        intensity_normalized = (intensity - np.min(intensity)) / (np.max(intensity) - np.min(intensity))
        
        colors = np.tile(intensity_normalized[:, None], (1, 3))
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        o3d.visualization.draw_geometries([pcd])

    def visualize_depth_image(self, image):
        plt.imshow(image,cmap='gray')
        plt.title('Depth Image')
        plt.axis('off')
        plt.show()

    def visualize_rgb_image(self, image):
        plt.imshow(image)
        plt.title('RGB Image')
        plt.axis('off')
        plt.show()


def plot_accum_lidar():
    seq_name = "c4c_garage"
    run_name = 4
    root_kitti_dir = "/media/donceykong/edd/in_processing/kitti"
    formatted_run_name = f"{seq_name}_{run_name:02d}"
    directory_path = os.path.join(root_kitti_dir, seq_name, formatted_run_name)
    groundtruth_path = os.path.join(directory_path, "groundtruth")
    lidar_path = os.path.join(directory_path, "lidar")

    # Load timestamps
    poses_directory = os.path.join(lidar_path, "poses")
    pointcloud_directory = os.path.join(lidar_path, "pointclouds")
    lidar_timestamps_file = os.path.join(lidar_path, "timestamps.txt")

    # List of pose files sorted by timestamp
    pose_files = sorted(os.listdir(poses_directory))[::10]  # Get every 10th pose file

    # Accumulate all transformed point clouds
    accumulated_points = []
    all_intensities = []
    all_reflectivities = []

    # Initialize list for frames
    frames = []

    # Load the timestamps from file
    with open(lidar_timestamps_file, 'r') as f:
        lidar_timestamps = f.read().splitlines()

    for pose_file in pose_files:
        # Load the pose matrix
        pose_matrix_path = os.path.join(poses_directory, pose_file)
        pose_matrix = load_bin_file(pose_matrix_path, shape=(4, 4))

        # Create a coordinate frame for each pose
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=pose_matrix[:3, 3])
        frame.rotate(pose_matrix[:3, :3], center=pose_matrix[:3, 3])
        frames.append(frame)

        # Find the corresponding timestamp index
        timestamp_value = pose_file.replace('.bin', '').replace('_', '.')
        timestamp_index = lidar_timestamps.index(timestamp_value) + 1

        pointcloud_file = f"lidar_pointcloud_{timestamp_index}.bin"
        pointcloud_path = os.path.join(pointcloud_directory, pointcloud_file)

        # Load the point cloud
        if os.path.exists(pointcloud_path):
            # Assuming point cloud has columns: x, y, z, intensity, reflectivity
            point_cloud = load_bin_file(pointcloud_path, shape=(-1, 5)) 
            point_cloud_xyz = point_cloud[:, :3]
            point_cloud_intensity = point_cloud[:, 3]
            point_cloud_reflectivity = point_cloud[:, 4]

            all_intensities.extend(point_cloud_intensity)
            all_reflectivities.extend(point_cloud_reflectivity)

            # Transform the point cloud using the pose matrix
            transformed_points = transform_point_cloud(point_cloud_xyz, pose_matrix)

            # Accumulate transformed points
            accumulated_points.append(transformed_points)
        else:
            print(f"Point cloud file {pointcloud_file} not found.")

    # Combine all accumulated points into a single array
    if accumulated_points:
        all_points = np.vstack(accumulated_points)
        all_intensities = np.hstack(all_intensities)
        all_reflectivities = np.hstack(all_reflectivities)
        all_z_values = all_points[:, 2]

        # Visualize using Open3D
        point_cloud_o3d = o3d.geometry.PointCloud()
        point_cloud_o3d.points = o3d.utility.Vector3dVector(all_points)

        # Normalize intensity and reflectivity using mean and standard deviation
        intensity_mean = np.mean(all_intensities)
        intensity_std = np.std(all_intensities)
        reflectivity_mean = np.mean(all_reflectivities)
        reflectivity_std = np.std(all_reflectivities)
        z_mean = np.mean(all_z_values)
        z_std = np.std(all_z_values)

        # Normalize using the standard deviation
        intensity_normalized = (all_intensities - intensity_mean) / intensity_std
        reflectivity_normalized = (all_reflectivities - reflectivity_mean) / reflectivity_std
        z_normalized = (all_z_values - z_mean) / z_std

        # Clip to range [0, 1] for visualization purposes
        z_normalized = np.clip(z_normalized, 0, 1)
        intensity_normalized = np.clip(intensity_normalized, 0, 1)
        reflectivity_normalized = np.clip(reflectivity_normalized, 0, 1)

        # Combine intensity and reflectivity for RGB mapping
        # Option 1: Average intensity and reflectivity
        combined_attribute = (intensity_normalized + reflectivity_normalized + z_normalized) / 3
        
        # Option 2: Different channels
        # combined_attribute = np.stack((intensity_normalized, reflectivity_normalized, reflectivity_normalized), axis=1)

        # Use a colormap to map combined attributes to RGB colors
        colormap = plt.get_cmap('viridis')  # Choose a colormap: 'viridis', 'jet', etc.
        colors = colormap(combined_attribute)[:, :3]  # Ignore the alpha channel
        point_cloud_o3d.colors = o3d.utility.Vector3dVector(colors)

        # Downsample the point cloud using voxel downsampling
        voxel_size = 0.2  # Adjust this value to control downsampling resolution
        downsampled_point_cloud = point_cloud_o3d.voxel_down_sample(voxel_size=voxel_size)

        # Add coordinate frames to the scene
        geometry_list = [downsampled_point_cloud] + frames

        # Visualize using Open3D
        o3d.visualization.draw_geometries(geometry_list,
                                        window_name='Downsampled Accumulated Point Cloud with Pose Frames',
                                        width=800, height=600)
    else:
        print("No point clouds found to accumulate.")