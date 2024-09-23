#!/usr/bin/env python3
"""
ColoRadar Processing Tools - crpt_rosbag_parser.py

Author: Doncey Albin
"""

import os
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import CameraInfo

class BagParser:
    def __init__(self, visualize_rosbag_data, log_paths_dict, visulizer = None):
        self.visualize_rosbag_data = visualize_rosbag_data
        self.visualizer = visulizer
        
        # Set paths
        self.lidar_path = log_paths_dict['lidar_path']
        self.lidar_pc_bin_path = log_paths_dict['lidar_pc_bin_path']
        self.camera_path = log_paths_dict['camera_path']
        self.camera_rgb_path = log_paths_dict['camera_rgb_path']
        self.camera_depth_path = log_paths_dict['camera_depth_path']
        self.groundtruth_path = log_paths_dict['groundtruth_path']

        # Set topics
        self.ouster_points_topic = "/ouster/points"
        self.imu_data_topic = "/gx5/imu/data"
        self.odom_topic = "/lio_sam/mapping/odometry"
        self.camera_depth_image_topic = '/camera/depth/image_rect_raw'
        self.camera_depth_info_topic = '/camera/depth/camera_info'
        self.camera_rgb_image_topic = "/camera/color/image_raw"
        self.camera_rgb_info_topic = '/camera/color/camera_info'
        self.heatmap_topic = "/cascade/heatmap"

        # Dictionary that maps topics to their respective handling functions. Keys can be used as wanted topics.
        self.topics_handlers_dict = {
            self.ouster_points_topic: self.handle_ouster_pointcloud,
            # self.imu_data_topic,
            self.camera_rgb_image_topic: self.handle_rgb_image,
            self.camera_depth_image_topic: self.handle_depth_image,
            self.odom_topic: self.handle_odometry,
            # self.heatmap_topic,
        }

        # Initialize number of data to 1 so that it corresponds with line in timestamp text file
        self.lidar_pc_num = 1
        self.camera_rgb_num = 1
        self.camera_depth_num = 1
        self.heatmap_num = 1

        # Initialize dictionaries for timestamps and data, where dict[timestamp] = data
        self.ouster_ts_index_dict = {}
        # self.imu_ts_data_dict = {}        
        self.odom_4x4_ts_data_dict = {}
        self.odom_quat_ts_data_dict = {}
        self.camera_depth_ts_index_dict = {}   
        self.camera_rgb_ts_index_dict = {}
        # self.heatmap_ts_index_dict = {}

    def handle_ouster_pointcloud(self, msg, msg_time):
        # Decode the point cloud
        field_names = ['x', 'y', 'z', 'intensity', 'reflectivity']
        points = pc2.read_points(msg, field_names=field_names, skip_nans=True)
        pointcloud = np.array(list(points), dtype=np.float32)
        # Save point cloud
        pointcloud_filename = f"{self.lidar_pc_bin_path}/unsorted_lidar_pointcloud_{self.lidar_pc_num}.bin"
        pointcloud.tofile(pointcloud_filename)
        # Store timestamp and index
        self.ouster_ts_index_dict[msg_time] = self.lidar_pc_num
        self.lidar_pc_num += 1
        # Visualize
        if self.visualize_rosbag_data and self.visualizer:
            self.visualizer.visualize_pointcloud(pointcloud)

    def handle_rgb_image(self, msg, msg_time):
        # Decode the rgb image: Convert the image data to a numpy array and reshape it.
        if msg.encoding != 'rgb8': # RGB image
            raise ValueError(f"RGB encoding incorrect! Expected rgb8, got {msg.encoding}.")
        dtype = np.uint8
        channels = 3
        image = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
        # Save rgb image
        rgb_image_filename = f"{self.camera_rgb_path}/unsorted_camera_rgb_image_{self.camera_rgb_num}.bin"
        image.tofile(rgb_image_filename)
        # Store timestamp and index
        self.camera_rgb_ts_index_dict[msg_time] = self.camera_rgb_num
        self.camera_rgb_num += 1
        # Visualize
        if self.visualize_rosbag_data and self.visualizer:
                self.visualizer.visualize_rgb_image(image)

    def handle_depth_image(self, msg, msg_time):
        # Decode the depth image: Convert the image data to a numpy array and reshape it.
        if msg.encoding != '16UC1': # Monochrome depth image
            raise ValueError(f"Depth image encoding incorrect! Expected 16UC1, got {msg.encoding}.")
        dtype = np.uint16
        channels = 1
        image = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
        # Save depth image
        depth_image_filename = f"{self.camera_depth_path}/unsorted_camera_depth_image_{self.camera_depth_num}.bin"
        image.tofile(depth_image_filename)
        # Store timestamp and index
        self.camera_depth_ts_index_dict[msg_time] = self.camera_depth_num
        self.camera_depth_num += 1
        # Visualize
        if self.visualize_rosbag_data and self.visualizer:
                self.visualizer.visualize_depth_image(image)

    def quaternion_to_4x4(self, msg):
        odom_4x4 = np.zeros((4, 4))
        odom_4x4[0, 3] = msg.pose.pose.position.x
        odom_4x4[1, 3] = msg.pose.pose.position.y
        odom_4x4[2, 3] = msg.pose.pose.position.z
        odom_4x4[3, 3] = 1

        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        s = x*x + y*y + z*z + w*w

        if s == 0:
            R = np.eye(3)
        else:
            s = 2/s
            X = x*s
            Y = y*s
            Z = z*s
            wX = w*X
            wY = w*Y
            wZ = w*Z
            xX = x*X
            xY = x*Y
            xZ = x*Z
            yY = y*Y
            yZ = y*Z
            zZ = z*Z
            R = np.array([[1-(yY+zZ), xY-wZ, xZ+wY],
                        [xY+wZ, 1-(xX+zZ), yZ-wX],
                        [xZ-wY, yZ+wX, 1-(xX+yY)]])
        odom_4x4[:3, :3] = R
             
        return odom_4x4

    def handle_odometry(self, msg, msg_time):
        # Decode the odometry
        odom_4x4_flat = self.quaternion_to_4x4(msg).flatten()
        odom_quat = np.asarray([msg.pose.pose.position.x, 
                                msg.pose.pose.position.y, 
                                msg.pose.pose.position.z,
                                msg.pose.pose.orientation.x,
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z,
                                msg.pose.pose.orientation.w])
        # Store the odometry data with timestamp
        self.odom_4x4_ts_data_dict[msg_time] = odom_4x4_flat
        self.odom_quat_ts_data_dict[msg_time] = odom_quat

    def read_bag(self, rosbag_path):
        # Open the bag file
        bag = rosbag.Bag(rosbag_path)

        for topic, msg, bag_time in bag.read_messages(self.topics_handlers_dict.keys()):
            # print(f"Processing message from topic: {topic}")
            msg_time = f"{msg.header.stamp.secs}.{msg.header.stamp.nsecs}"

            # Dispatch message to the corresponding handler function
            self.topics_handlers_dict[topic](msg, msg_time)

        # Close the bag file
        bag.close()

    def write_data_to_files(self):
        # Helper function to handle timestamps, renaming, and saving data
        def save_and_rename(timestamp_file_prefix, ts_index_dict, timestamp_path, data_path, filename_prefix):
            timestamps_np = np.array(sorted(ts_index_dict.keys()))
            np.savetxt(f'{timestamp_path}/{timestamp_file_prefix}timestamps.txt', timestamps_np, fmt='%s')
            
            # print(f"Renaming {data_type} data!\n\n")
            for idx, timestamp in enumerate(timestamps_np):
                original_index = ts_index_dict[timestamp]
                new_index = idx + 1
                original_filename = f"{data_path}/unsorted_{filename_prefix}_{original_index}.bin"
                new_filename = f"{data_path}/{filename_prefix}_{new_index}.bin"
                os.rename(original_filename, new_filename)
        
        # Write and rename CAMERA RGB data
        save_and_rename("rgb_", self.camera_rgb_ts_index_dict, self.camera_path, self.camera_rgb_path, "camera_rgb_image")
        
        # Write and rename CAMERA DEPTH data
        save_and_rename("depth_", self.camera_depth_ts_index_dict, self.camera_path, self.camera_depth_path, "camera_depth_image")
        
        # Write and rename LIDAR data
        save_and_rename("", self.ouster_ts_index_dict, self.lidar_path, self.lidar_pc_bin_path, "lidar_pointcloud")
        
        # Write odometry timestamps and data
        odom_timestamps_np = np.array(sorted(self.odom_4x4_ts_data_dict.keys()))
        np.savetxt(f'{self.groundtruth_path}/timestamps.txt', odom_timestamps_np, fmt='%s')

        odom_4x4_np = np.array([self.odom_4x4_ts_data_dict[timestamp] for timestamp in odom_timestamps_np])
        np.savetxt(f'{self.groundtruth_path}/groundtruth_poses_4x4.txt', odom_4x4_np)

        odom_quat_np = np.array([self.odom_quat_ts_data_dict[timestamp] for timestamp in odom_timestamps_np])
        np.savetxt(f'{self.groundtruth_path}/groundtruth_poses_quat.txt', odom_quat_np)


    # def write_data_to_files(self):
    #     """_summary_

    #     Args:
    #         log_paths_dict (_type_): _description_
    #     """
    #     # Write CAMERA RGB timestamps and data
    #     rs_rgb_timestamps_np = np.array(sorted(self.camera_rgb_ts_index_dict.keys()))
    #     np.savetxt(f'{self.camera_path}/rgb_timestamps.txt', rs_rgb_timestamps_np, fmt='%s')

    #     print(f"Renaming camera rgb images!\n\n")
    #     for idx, timestamp in enumerate(rs_rgb_timestamps_np):
    #         rgb_image_index = self.camera_rgb_ts_index_dict[timestamp] # Original index
    #         new_rgb_image_index = idx + 1                              # Index after sorting timestamps
    #         original_filename = f"{self.camera_rgb_path}/unsorted_camera_rgb_image_{rgb_image_index}.bin"
    #         new_filename = f"{self.camera_rgb_path}/camera_rgb_image_{new_rgb_image_index}.bin"
    #         os.rename(original_filename, new_filename)  # Rename data bin file using sorted timestamp index

    #     # Write CAMERA DEPTH timestamps and data
    #     rs_depth_timestamps_np = np.array(sorted(self.camera_depth_ts_index_dict.keys()))
    #     np.savetxt(f'{self.camera_path}/depth_timestamps.txt', rs_depth_timestamps_np, fmt='%s')

    #     print(f"Renaming camera depth images!\n\n")
    #     for idx, timestamp in enumerate(rs_depth_timestamps_np):
    #         depth_image_index = self.camera_depth_ts_index_dict[timestamp]  # Original index
    #         new_depth_image_index = idx + 1                                 # Index after sorting timestamps
    #         original_filename = f"{self.camera_depth_path}/unsorted_camera_depth_image_{depth_image_index}.bin"
    #         new_filename = f"{self.camera_depth_path}/camera_depth_image_{new_depth_image_index}.bin"
    #         os.rename(original_filename, new_filename)  # Rename data bin file using sorted timestamp index

    #     # Write LIDAR timestamps and data
    #     ouster_timestamps_np = np.array(sorted(self.ouster_ts_index_dict.keys()))
    #     np.savetxt(f'{self.lidar_path}/timestamps.txt', ouster_timestamps_np, fmt='%s')

    #     print(f"Renaming lidar pointclouds!\n\n")
    #     for idx, timestamp in enumerate(ouster_timestamps_np):
    #         # Get the pointcloud index associated with the timestamp
    #         pc_index = self.ouster_ts_index_dict[timestamp]     # Original index
    #         new_pc_index = idx + 1                              # Index after sorting timestamps
    #         original_filename = f"{self.lidar_pc_bin_path}/pointcloud_{pc_index}.bin"       # Original file name
    #         new_filename = f"{self.lidar_pc_bin_path}/lidar_pointcloud_{new_pc_index}.bin"  # New filename based on sorted index
    #         os.rename(original_filename, new_filename)  # Rename data bin file using sorted timestamp index
    #         # print(f"Original index: {pc_index}, new index: {new_pc_index}")

    #     # Write odometry timestamps and data
    #     odom_timestamps_np = np.array(sorted(self.odom_4x4_ts_data_dict.keys()))
    #     np.savetxt(f'{self.groundtruth_path}/timestamps.txt', odom_timestamps_np, fmt='%s')

    #     sorted_odom_4x4_list = [self.odom_4x4_ts_data_dict[timestamp] for timestamp in odom_timestamps_np]
    #     odom_4x4_np = np.array(sorted_odom_4x4_list)
    #     np.savetxt(f'{self.groundtruth_path}/groundtruth_poses_4x4.txt', odom_4x4_np)

    #     sorted_odom_quat_list = [self.odom_quat_ts_data_dict[timestamp] for timestamp in odom_timestamps_np]
    #     odom_quat_np = np.array(sorted_odom_quat_list)
    #     np.savetxt(f'{self.groundtruth_path}/groundtruth_poses_quat.txt', odom_quat_np)