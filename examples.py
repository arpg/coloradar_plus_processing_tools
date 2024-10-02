#!/usr/bin/env python3

import os
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from decimal import Decimal
import open3d as o3d

from coloradar_plus_processing_tools import utils

def main():
    # Define the directories to search for bag files
    directory_path = "/media/donceykong/puck_of_destiny/datasets/coloradar_plus/bags"
    directories = ["irl", "c4c_garage", "regent_garage", "ec_hallways", "ec_courtyard"]
    odom_topic = "/lio_sam/mapping/odometry"
    output_csv_path = "./coloradar_plus_dataset_stats.csv"

    get_seq_stats(directory_path, directories, odom_topic, output_csv_path)

    # Check if CSV is written correctly
    if os.path.exists(output_csv_path) and os.path.getsize(output_csv_path) > 0:
        print(f"CSV file has been written successfully at {output_csv_path} with size {os.path.getsize(output_csv_path)} bytes.")
    else:
        print("CSV file has not been written or is empty.")

    # Optionally, print CSV contents
    with open(output_csv_path, mode='r') as file:
        print("\nCSV Content:")
        print(file.read())
        
def main():
    seq_name = "c4c_garage"
    run_name = 4
    root_kitti_dir = "/media/donceykong/edd/in_processing/kitti"
    formatted_run_name = f"{seq_name}_{run_name:02d}"
    directory_path = os.path.join(root_kitti_dir, seq_name, formatted_run_name)
    groundtruth_path = os.path.join(directory_path, "groundtruth")
    lidar_path = os.path.join(directory_path, "lidar")

    # Load timestamps
    ouster_lidar_timestamps_file = os.path.join(lidar_path, "timestamps.txt")
    pose_timestamps_file = os.path.join(groundtruth_path, "timestamps.txt")
    poses_file = os.path.join(groundtruth_path, "groundtruth_poses_quat.txt")

    # Directory to save the pose matrices
    output_directory = os.path.join(lidar_path, "poses")
    os.makedirs(output_directory, exist_ok=True)

    pose_timestamps_str = np.loadtxt(pose_timestamps_file, dtype=str)
    pose_timestamps = [Decimal(ts) for ts in pose_timestamps_str]   # Convert to Decimal for high precision
    poses_data = np.loadtxt(poses_file)

    # # If using 4x4 poses, ensure pose_data is reshaped correctly into 4x4 matrices
    # poses = poses_data.reshape(-1, 4, 4)
    poses = poses_data

    positions = []
    quaternions = []

    for pose in poses:
        pos = pose[:3]
        quat = pose[3:]

        # # If using 4x4 pose
        # pos, quat = utils.quat_from_4x4(pose)

        positions.append(pos)
        quaternions.append(quat)

    positions = np.array(positions)
    quaternions = np.array(quaternions)

    min_timestamp = min(pose_timestamps)
    max_timestamp = max(pose_timestamps)

    print(f"pose_timestamps[0]: {pose_timestamps[0]}, pose_timestamps[-1]: {pose_timestamps[-1]}")
    print(f"min_timestamp: {min_timestamp}, max_timestamp: {max_timestamp}")

    # Load and filter ouster timestamps
    ouster_timestamps_str = np.loadtxt(ouster_lidar_timestamps_file, dtype=str)
    ouster_timestamps = [Decimal(ts) for ts in ouster_timestamps_str]   # Convert to Decimal for high precision

    # Filter ouster timestamps to find those within the min-max range
    ouster_filtered_timestamps = [ts for ts in ouster_timestamps if min_timestamp <= ts <= max_timestamp]

    # Interpolate poses for all ouster timestamps
    interp_position_list = []
    interp_orientation_list = []

    for ouster_timestamp in ouster_filtered_timestamps:
        print(f"Interpolating for ouster timestamp: {ouster_timestamp}")
        interp_position, interp_orientation = utils.interpolate_pose(pose_timestamps, positions, quaternions, ouster_timestamp)
        if interp_position is not None and interp_orientation is not None:
            interp_position_list.append(interp_position)
            interp_orientation_list.append(interp_orientation)
            
            # Create a 4x4 pose matrix
            pose_matrix = utils.quat_to_4x4(interp_position, interp_orientation)
            
            # Save the pose matrix as a binary file with underscores in the timestamp
            timestamp_str = str(ouster_timestamp).replace('.', '_')
            bin_filename = os.path.join(output_directory, f"{timestamp_str}.bin")
            utils.save_pose_as_bin(pose_matrix, bin_filename)
            print(f"Saved interpolated pose as {bin_filename}")

    # Convert lists to numpy arrays for visualization
    interp_positions = np.array(interp_position_list)
    interp_orientations = np.array(interp_orientation_list)
        
    interp_positions = interp_positions[:1000]
    interp_orientations = interp_orientations[:1000]
    positions = positions[:500]
    quaternions = quaternions[:500]

    # Visualize the original and interpolated positions
    utils.visualize_positions(positions, interp_positions)
    utils.visualize_frames(positions, interp_positions, quaternions, interp_orientations)

if __name__ == "__main__":
    main()
