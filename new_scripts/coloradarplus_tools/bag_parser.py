#!/usr/bin/env python3
"""

"""

import numpy as np
import struct
import matplotlib.pyplot as plt

# ROS-based imports
import rosbag
import sensor_msgs.point_cloud2 as pc2

class BagParser:
    def __init__(self, visualize_rosbag_data, log_paths_dict, visulizer = None):
        self.visualize_rosbag_data = visualize_rosbag_data
        self.log_paths_dict = log_paths_dict
        self.visualizer = visulizer
        
        # Set topics
        self.ouster_points_topic = "/ouster/points"
        self.odom_topic = "/lio_sam/mapping/odometry"
        self.depth_image_topic = '/camera/depth/image_rect_raw'
        self.rgb_image_topic = "/camera/color/image_raw"
        self.topics = [self.ouster_points_topic, 
                       self.odom_topic, 
                       self.depth_image_topic, 
                       self.rgb_image_topic,
                       ]

        # Dictionaries for timestamps and data
        self.ouster_timestamps = []
        self.ouster_timestamp_pc_dict = {}
        self.odom_4x4_data_dict = {}
        self.odom_quat_data_dict = {}
        self.rs_depth_timestamp_list = []
        self.rs_rgb_timestamp_list = []

    def decode_pointcloud2(self, cloud):
        field_names = ['x', 'y', 'z', 'intensity', 'reflectivity']
        points = pc2.read_points(cloud, field_names=field_names, skip_nans=True)
        points_array = np.array(list(points))
        
        if self.visualize_rosbag_data and self.visualizer:
            self.visualizer.visualize_pointcloud(points_array)

        return points_array

    def decode_rgb_image(self, msg):
        if msg.encoding != 'rgb8': # RGB image
            raise ValueError(f"RGB encoding incorrect! Expected rgb8, got {msg.encoding}.")
        
        # Convert the image data to a numpy array and reshape it
        dtype = np.uint8
        channels = 3
        image = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)

        if self.visualize_rosbag_data and self.visualizer:
                self.visualizer.visualize_rgb_image(image)

        return image
    
    def decode_depth_image(self, msg):
        """
        Decode a ROS sensor_msgs/Image message into a numpy array.
        """
        if msg.encoding != '16UC1': # Monochrome depth image
            raise ValueError(f"Depth image encoding incorrect! Expected 16UC1, got {msg.encoding}.")
        
        # Convert the image data to a numpy array and reshape it
        dtype = np.uint16
        channels = 1
        image = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)

        if self.visualize_rosbag_data and self.visualizer:
                self.visualizer.visualize_depth_image(image)

        return image
    
    def decode_odom(self, msg):
        odom_mat = np.zeros((4, 4))
        odom_mat[0, 3] = msg.pose.pose.position.x
        odom_mat[1, 3] = msg.pose.pose.position.y
        odom_mat[2, 3] = msg.pose.pose.position.z
        odom_mat[3, 3] = 1

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
        odom_mat[:3, :3] = R
        odom_4x4_flat = odom_mat.flatten()
        odom_quat = np.asarray([msg.pose.pose.position.x, 
                                msg.pose.pose.position.y, 
                                msg.pose.pose.position.z,
                                msg.pose.pose.orientation.x,
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z,
                                msg.pose.pose.orientation.w])
             
        return odom_4x4_flat, odom_quat
    
    def read_bag(self, rosbag_path):
        # Open the bag file
        bag = rosbag.Bag(rosbag_path)
        lidar_pc_num = 1
        camera_rgb_num = 1
        camera_depth_num = 1

        for topic, msg, bag_time in bag.read_messages(self.topics):
            print(f"Processing message from topic: {topic}")
            msg_time = f"{msg.header.stamp.secs}.{msg.header.stamp.nsecs}"
            
            if topic == self.ouster_points_topic:
                # Decode the point cloud
                pointcloud = self.decode_pointcloud2(msg)
                pointcloud = np.array(list(pointcloud), dtype=np.float32)
                # pointcloud_filename = f"{log_paths_dict['lidar_pc_bin_path']}/{msg_time.replace('.', '_')}.bin"
                pointcloud_filename = f"{self.log_paths_dict['lidar_pc_bin_path']}/lidar_pointcloud_{lidar_pc_num}.bin"
                pointcloud.tofile(pointcloud_filename)
                self.ouster_timestamps.append(msg_time)
                self.ouster_timestamp_pc_dict[msg_time] = lidar_pc_num  # TODO: Used later on after sorting timestamps
                lidar_pc_num += 1
            if topic == self.rgb_image_topic:
                    image = self.decode_rgb_image(msg)
                    rgb_image_filename = f"{self.log_paths_dict['camera_rgb_path']}/camera_rgb_image_{camera_rgb_num}.bin"
                    image.tofile(rgb_image_filename)
                    self.rs_rgb_timestamp_list.append(msg_time)
                    camera_rgb_num += 1
            if topic == self.depth_image_topic:
                    image = self.decode_depth_image(msg)
                    image.tofile(f"{self.log_paths_dict['camera_depth_path']}/camera_depth_image_{camera_depth_num}.bin")
                    self.rs_depth_timestamp_list.append(msg_time)
                    camera_depth_num += 1
            if topic == self.odom_topic:
                # Decode the odometry
                odom_flat, odom_quat = self.decode_odom(msg)
                self.odom_4x4_data_dict[msg_time] = odom_flat
                self.odom_quat_data_dict[msg_time] = odom_quat

        self.write_data_to_files()

        # Close the bag file
        bag.close()

    def write_data_to_files(self):
        """_summary_

        Args:
            log_paths_dict (_type_): _description_
        """
        # Write Realsense timestamps and data
        rs_rgb_timestamps_np = sorted(self.rs_rgb_timestamp_list)
        rs_depth_timestamps_np = sorted(self.rs_depth_timestamp_list)
        np.savetxt(f'{self.log_paths_dict['camera_path']}/rgb_timestamps.txt', self.rs_rgb_timestamps_np, fmt='%s')
        np.savetxt(f'{self.log_paths_dict['camera_path']}/depth_timestamps.txt', self.rs_depth_timestamps_np, fmt='%s')

        # Write ouster timestamps and data
        sorted_ouster_timestamps = sorted(self.ouster_timestamps)
        ouster_timestamps_np = np.array(sorted_ouster_timestamps)
        np.savetxt(f'{self.log_paths_dict['lidar_path']}/timestamps.txt', ouster_timestamps_np, fmt='%s')
        #   TODO: Make sure ouster pointcloud bin files are renamed wrt the line their timestamp falls on timestamp txt file
        #   TODO: Use self.ouster_timestamp_pc_dict

        # Write odometry timestamps and data
        sorted_odom_timestamps = sorted(self.odom_4x4_data_dict.keys())
        odom_timestamps_np = np.array(sorted_odom_timestamps)
        np.savetxt(f'{self.log_paths_dict['groundtruth_path']}/timestamps.txt', odom_timestamps_np, fmt='%s')

        sorted_odom_4x4_list = [self.odom_4x4_data_dict[timestamp] for timestamp in sorted_odom_timestamps]
        odom_4x4_np = np.array(sorted_odom_4x4_list)
        np.savetxt(f'{self.log_paths_dict['groundtruth_path']}/groundtruth_poses_4x4.txt', odom_4x4_np)

        sorted_odom_quat_list = [self.odom_quat_data_dict[timestamp] for timestamp in sorted_odom_timestamps]
        odom_quat_np = np.array(sorted_odom_quat_list)
        np.savetxt(f'{self.log_paths_dict['groundtruth_path']}/groundtruth_poses_quat.txt', odom_quat_np)