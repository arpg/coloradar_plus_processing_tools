#!/usr/bin/env python3

import os
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


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
    plt.imshow(image)
    plt.title('RGB Image')
    plt.axis('off')
    plt.show()


def visualize_positions(original_positions, interpolated_positions):
    # Identify non-matching interpolated points
    interpolated_set = set(map(tuple, interpolated_positions))
    original_set = set(map(tuple, original_positions))
    unique_interpolated_positions = np.array([pos for pos in interpolated_set if pos not in original_set])

    # Create Open3D point clouds for unique interpolated positions
    unique_interpolated_pcd = o3d.geometry.PointCloud()
    unique_interpolated_pcd.points = o3d.utility.Vector3dVector(unique_interpolated_positions)
    unique_interpolated_pcd.paint_uniform_color([1, 0, 0])  # Red for unique interpolated

    # Set points for original positions
    original_pcd = o3d.geometry.PointCloud()
    original_pcd.points = o3d.utility.Vector3dVector(original_positions)
    original_pcd.paint_uniform_color([0, 0, 1])  # Blue for original

    # Visualize the unique interpolated points
    o3d.visualization.draw_geometries([unique_interpolated_pcd, original_pcd],
                                      window_name='Unique Interpolated Positions',
                                      width=800, height=600)


def visualize_trajectory_with_orientation(odom_list):
    # Create open3d LineSet for trajectory visualization
    points = np.array([(odom[0, 3], odom[1, 3], odom[2, 3]) for odom in odom_list])
    lines = [[i, i + 1] for i in range(len(points) - 1)]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.paint_uniform_color([0, 0, 1])  # Set line color to blue

    # Create coordinate frames for orientation visualization
    frames = []
    for odom in odom_list:
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        frame.transform(odom)
        frames.append(frame)

    # Visualize using open3d
    o3d.visualization.draw_geometries([line_set, *frames], window_name='Odometry Trajectory with Orientation',
                                      zoom=0.5, front=[0.5, -0.5, -0.5], lookat=[0, 0, 0], up=[0, 1, 0])
    

def visualize_frames(original_positions, interpolated_positions, original_orientations, interpolated_orientations):
    # Identify non-matching interpolated frames
    interpolated_set = set(map(tuple, interpolated_positions))
    original_set = set(map(tuple, original_positions))
    unique_interpolated_positions = np.array([pos for pos in interpolated_set if pos not in original_set])

    # Create Open3D geometries for unique interpolated frames (red)
    unique_interpolated_frames = []
    for pos, orient in zip(unique_interpolated_positions, interpolated_orientations):
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        frame.translate(pos)
        rotation_matrix = o3d.geometry.get_rotation_matrix_from_quaternion(orient)
        frame.rotate(rotation_matrix, center=pos)
        unique_interpolated_frames.append(frame)
    
    # Set frames for original positions (blue)
    original_frames = []
    for pos, orient in zip(original_positions, original_orientations):
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        frame.translate(pos)
        rotation_matrix = o3d.geometry.get_rotation_matrix_from_quaternion(orient)
        frame.rotate(rotation_matrix, center=pos)
        original_frames.append(frame)

    # Set color for the frames
    for frame in unique_interpolated_frames:
        frame.paint_uniform_color([1, 0, 0])  # Red for unique interpolated frames
    for frame in original_frames:
        frame.paint_uniform_color([0, 0, 1])  # Blue for original frames

    # Visualize the frames
    o3d.visualization.draw_geometries(unique_interpolated_frames + original_frames,
                                      window_name='Unique Interpolated and Original Frames',
                                      width=800, height=600)


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