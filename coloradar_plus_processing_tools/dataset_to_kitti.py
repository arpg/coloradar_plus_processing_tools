#!/usr/bin/env python3

import os
import numpy as np
import shutil

# Internal Imports
from coloradar_plus_processing_tools import BagParser
#from coloradar_plus_processing_tools import Visualizer

class DatasetToKitti:
    def __init__(self, crp_config_dict = None):
        self.crp_config_dict = crp_config_dict 
        
    def convert(self):
        """_Converts bags specified in config file to KITTI-style unstructured dataset._
        """
        # Extract data from runs in each sequence specified in config
        for seq_name in self.crp_config_dict["sequences"]:
            print(f"\n\nSequence: {seq_name}")
            sequence = self.crp_config_dict["sequences"][f"{seq_name}"]
            for run_name in sequence["runs"]:
                self.bag_to_kitti(seq_name, run_name)

    def bag_to_kitti(self, seq_name, run_name):
        print(f"    * run: {run_name}")

        # Create KITTI-style directory for run
        kitti_paths_dict = self.create_run_kitti_directory(seq_name=seq_name, run_name=run_name)
        kitti_directory_path = kitti_paths_dict['directory_path']
        print(f"        - Created KITTI-style directory: {kitti_directory_path}")

        # Init bagparser object
        bag_parser = BagParser(log_paths_dict=kitti_paths_dict)

        # Parse rosbag data
        bag_path = os.path.join(self.crp_config_dict["main_bag_directory_path"], seq_name, f"{seq_name}_run{run_name:01d}.bag")
        print(f"        - Parsing rosbag: {bag_path}")
        bag_parser.read_bag(rosbag_path=bag_path)

        # Save data
        bag_parser.write_data_to_files()

        # Compress kitti-style directory and delete uncompressed
        print(f"        - Compressing and removing uncompressed directory: {kitti_directory_path}")
        compressed_path = shutil.make_archive(kitti_directory_path, 'zip', kitti_directory_path)
        shutil.rmtree(kitti_directory_path)
        print(f"        - Run {run_name} processed and compressed to {compressed_path}")

    def create_run_kitti_directory(self, seq_name, run_name):
        # Make sure base directory for run exists or create it
        root_kitti_dir = self.crp_config_dict["main_kitti_directory_path"]
        formatted_run_name = f"{seq_name}_run{run_name:01d}"
        directory_path = os.path.join(root_kitti_dir, seq_name, formatted_run_name)

        # Dictionary to hold all necessary subdirectory paths
        run_dir_paths_dict = {
            'directory_path': directory_path,
            'calib_trans_dir_path': os.path.join(root_kitti_dir, "calib", "transforms"),
            'calib_cascade_dir_path': os.path.join(root_kitti_dir, "calib", "cascade"),
            'cascade_heatmap_path': os.path.join(directory_path, "cascade", "heatmaps"),
            'cascade_heatmap_data_path': os.path.join(directory_path, "cascade", "heatmaps", "data"),
            'cascade_datacube_path': os.path.join(directory_path, "cascade", "adc_samples"),
            'cascade_datacube_data_path': os.path.join(directory_path, "cascade", "adc_samples", "data"),
            'groundtruth_path': os.path.join(directory_path, "groundtruth"),
            'imu_path': os.path.join(directory_path, "imu"),
            'lidar_path': os.path.join(directory_path, "lidar"),
            'lidar_pc_bin_path': os.path.join(directory_path, "lidar", "pointclouds"),
            'camera_path': os.path.join(directory_path, "camera"),
            'camera_rgb_path': os.path.join(directory_path, "camera", "images", "rgb"),
            'camera_depth_path': os.path.join(directory_path, "camera", "images", "depth")
        }

        # Create all directories based on the paths in the dictionary
        for path in run_dir_paths_dict.values():
            os.makedirs(path, exist_ok=True)

        return run_dir_paths_dict