#!/usr/bin/env python3
"""

"""

import os
import numpy as np

from coloradarplus_tools import BagParser
from coloradarplus_tools import Visualizer

class DatasetToKitti:
    def __init__(self, crp_config_dict = None):
        self.crp_config_dict = crp_config_dict

        # Init visualizer object
        self.visualizer = Visualizer()

    def convert(self):
        """_Converts bags specified in config file to KITTI-style unstructured dataset._
        """
        # Extract data from runs in each sequence specified in config
        for seq_name in self.crp_config_dict["sequences"]:
            sequence = self.crp_config_dict["sequences"][f"{seq_name}"]
            for run_name in sequence["runs"]:
                self.bag_to_kitti(seq_name, run_name)

    def bag_to_kitti(self, seq_name, run_name):
        print(f"seq_name: {seq_name}, run_name: {run_name}")

        # Create KITTI-style directory for run
        kitti_paths_dict = self.create_run_kitti_directory(seq_name=seq_name, run_name=run_name)

        # Init bagparser object
        visualize_rosbag_data = self.crp_config_dict["display_rosbag_data"]
        bag_parser = BagParser(visualize_rosbag_data=visualize_rosbag_data, 
                               log_paths_dict=kitti_paths_dict,
                               visulizer=self.visualizer)

        # Extract data
        root_bag_dir = self.crp_config_dict["main_bag_directory_path"]
        bag_path = os.path.join(root_bag_dir, seq_name, f"{seq_name}_{run_name:02d}.bag")
        bag_parser.read_bag(rosbag_path=bag_path)

        # Save data
        bag_parser.write_data_to_files(log_paths_dict = kitti_paths_dict)

    def create_run_kitti_directory(self, seq_name, run_name):
        # Make sure base directory for run exists or create it
        root_kitti_dir = self.crp_config_dict["main_kitti_directory_path"]
        formatted_run_name = f"{seq_name}_{run_name:02d}"
        directory_path = os.path.join(root_kitti_dir, seq_name, formatted_run_name)

        # Dictionary to hold all necessary subdirectory paths
        run_dir_paths_dict = {
            'cascade_path': os.path.join(directory_path, "cascade", "heatmaps"),
            'groundtruth_path': os.path.join(directory_path, "groundtruth"),
            'imu_path': os.path.join(directory_path, "imu"),
            'lidar_path': os.path.join(directory_path, "lidar", "pointclouds"),
            'lidar_pc_bin_path': os.path.join(directory_path, "lidar", "pointclouds"),
            'camera_path': os.path.join(directory_path, "camera"),
            'camera_rgb_path': os.path.join(directory_path, "camera", "images", "rgb"),
            'camera_depth_path': os.path.join(directory_path, "camera", "images", "depth")
        }

        # Create all directories based on the paths in the dictionary
        for path in run_dir_paths_dict.values():
            os.makedirs(path, exist_ok=True)

        print(f"Directory and subdirectories ensured: {directory_path}")

        return run_dir_paths_dict