#!/usr/bin/env python3

import os
import numpy as np
np.set_printoptions(suppress=True)
from array import array
import rosbag
from sensor_msgs.msg import CameraInfo
import json
import cv2
from coloradar_plus_processing_tools import utils


class BagParser:
    def __init__(self, log_paths_dict):
        # Set paths for run
        for key, value in log_paths_dict.items():
            setattr(self, key, value)

        # Set topics
        self.ouster_points_topic = "/ouster/points"
        self.imu_data_topic = "/gx5/imu/data"
        self.odom_topic = "/lio_sam/mapping/odometry"
        self.path_topic = "/lio_sam/mapping/path" 
        self.camera_depth_image_topic = '/camera/depth/image_rect_raw'
        self.camera_depth_info_topic = '/camera/depth/camera_info'
        self.camera_rgb_image_topic = "/camera/color/image_raw"
        self.camera_rgb_info_topic = '/camera/color/camera_info'
        self.cascade_datacube_topic = "/cascade/data_cube"
        self.cascade_heatmap_topic = "/cascade/heatmap"
        self.transforms_topic = "/tf"
        self.static_transforms_topic = "/tf_static"

        
        # Dictionary that maps topics to their respective handling functions. Keys can be used as wanted topics.
        self.topics_handlers_dict = {
            self.ouster_points_topic: self.handle_ouster_pointcloud,
            self.imu_data_topic: self.handle_imu_data,
            self.camera_rgb_image_topic: self.handle_rgb_image,
            self.camera_rgb_info_topic:self.handle_rgb_cam_info, 
            self.camera_depth_image_topic: self.handle_depth_image,
            self.camera_depth_info_topic: self.handle_depth_cam_info, 
            self.odom_topic: self.handle_odometry,
            self.path_topic: self.handle_path, 
            self.cascade_datacube_topic: self.handle_cascade_datacube,
            self.cascade_heatmap_topic: self.handle_cascade_heatmap,
            # self.transforms_topic: self.handle_transforms,
            self.static_transforms_topic: self.handle_transforms,
        }

        # Initialize number of data to 1 so that it corresponds with line in timestamp text file
        self.lidar_pc_num = 0
        self.camera_rgb_num = 0
        self.camera_depth_num = 0
        self.cascade_heatmap_num = 0
        self.cascade_datacube_num = 0

        # Initilize tfs
        self.base_to_rig_tf = False
        self.rig_to_cascade_tf = False
        self.rig_to_camera_tf = False
        self.rig_to_mountplate_tf = False
        self.mountplate_to_lidar_tf = False
        self.lidar_to_imu_tf = False

        # Initialize dictionaries for timestamps and data, where dict[timestamp] = data
        self.ouster_ts_index_dict = {}
        self.imu_ts_data_dict = {}        
        self.odom_4x4_ts_data_dict = {}
        self.odom_quat_ts_data_dict = {}
        self.fin_path_4x4_ts_data_dict = {}
        self.fin_path_quat_ts_data_dict = {}
        self.camera_depth_ts_index_dict = {}   
        self.camera_rgb_ts_index_dict = {}
        self.cascade_heatmap_ts_index_dict = {}
        self.cascade_datacube_ts_index_dict = {}

        self.path_msg = None
        self.rgb_info_initted = False 
        self.rgb_info = None 
        self.depth_info_initted = False  
        self.depth_info =  None 

    def handle_ouster_pointcloud(self, msg, msg_header_time):
        pointcloud = utils.pointcloud_msg_to_numpy(msg)

        # Save point cloud
        pointcloud_filename = f"{self.lidar_pc_bin_path}/unsorted_lidar_pointcloud_{self.lidar_pc_num}.bin"
        pointcloud.tofile(pointcloud_filename)

        # Store timestamp and index
        self.ouster_ts_index_dict[msg_header_time] = self.lidar_pc_num
        self.lidar_pc_num += 1

    def handle_rgb_cam_info(self, msg, msg_header_time):
        if not self.rgb_info_initted: 
            self.rgb_info_initted = True 
            self.rgb_info = utils.cam_info_to_json(msg)

    def handle_rgb_image(self, msg, msg_header_time):
        image = utils.image_msg_to_numpy(msg=msg)
        if image.dtype != np.uint8:
            image = (255 * (image - np.min(image)) / (np.max(image) - np.min(image))).astype(np.uint8)

        filename = f"{self.camera_rgb_path}/unsorted_camera_rgb_image_{self.camera_rgb_num}.png"
        cv2.imwrite(filename, image)

        # Store timestamp and index
        self.camera_rgb_ts_index_dict[msg_header_time] = self.camera_rgb_num
        self.camera_rgb_num += 1

    def handle_depth_cam_info(self,msg,msg_header_time):
        if not self.depth_info_initted: 
            self.depth_info_initted = True 
            self.depth_info = utils.cam_info_to_json(msg) 

    def handle_depth_image(self, msg, msg_header_time):
        image = utils.image_msg_to_numpy(msg, datatype=np.uint16, num_channels=1)

        # Save depth image
        filename = f"{self.camera_depth_path}/unsorted_camera_depth_image_{self.camera_depth_num}.bin"
        image.tofile(filename)

        # Store timestamp and index
        self.camera_depth_ts_index_dict[msg_header_time] = self.camera_depth_num
        self.camera_depth_num += 1

    def handle_path(self, msg, msg_header_time=None): 
        self.path_msg = msg

    def handle_odometry(self, msg, msg_header_time):
        odom_4x4_flat, odom_quat_flat = utils.pose_msg_to_numpy(msg.pose)

        self.odom_4x4_ts_data_dict[msg_header_time] = odom_4x4_flat
        self.odom_quat_ts_data_dict[msg_header_time] = odom_quat_flat
        # gt_data_file.write(str(p_x) + ' ' + str(p_y) + ' ' + str(p_z) + ' ' + str(q_x) + ' ' + str(q_y) + ' ' + str(q_z) + ' ' + str(q_w) + '\n')

    def handle_imu_data(self, msg, msg_header_time):
        # Decode IMU
        imu_numpy = utils.imu_msg_to_numpy(msg)
        self.imu_ts_data_dict[msg_header_time] = imu_numpy[:6]  # Do not save orientation data


    def handle_cascade_heatmap(self, msg, msg_header_time):
        # # Generate config file if not yet exists
        # cascade_heatmap_config_file_path = os.path.join(self.calib_cascade_dir_path, 'heatmap_cfg.txt')
        # if not os.path.exists(cascade_heatmap_config_file_path):
        #     #self.save_cascade_heatmap_config(msg)
        #     utils.save_cascade_heatmap_config(msg,cascade_heatmap_config_file_path)

        # Decode the heatmap
        cascade_heatmap_array = np.array(msg.image, dtype=np.float32)

        # Save cascade heatmap
        cascade_heatmap_filename = f"{self.cascade_heatmap_data_path}/unsorted_heatmap_{self.cascade_heatmap_num}.bin"
        cascade_heatmap_array.tofile(cascade_heatmap_filename)
        self.cascade_heatmap_ts_index_dict[msg_header_time] = self.cascade_heatmap_num
        self.cascade_heatmap_num += 1

    def handle_cascade_datacube(self, msg, msg_header_time):
        # # Generate config file if not yet exists
        # cascade_datacube_config_file_path = os.path.join(self.calib_cascade_dir_path, 'waveform_cfg.txt')
        # if not os.path.exists(cascade_datacube_config_file_path):
        #     utils.save_cascade_datacube_config(msg,cascade_datacube_config_file_path)

        # Decode the datacube
        cascade_datacube_array = np.array(msg.samples, dtype=np.int16)

        # Save cascade datacube
        cascade_datacube_filename = f"{self.cascade_datacube_data_path}/unsorted_frame_{self.cascade_datacube_num}.bin"
        cascade_datacube_array.tofile(cascade_datacube_filename)
        self.cascade_datacube_ts_index_dict[msg_header_time] = self.cascade_datacube_num
        self.cascade_datacube_num += 1

    def handle_transforms(self, msg, msg_time):
        if hasattr(msg, 'transforms'):
            for transform in msg.transforms:
                filename = None

                if self.base_to_rig_tf and self.rig_to_cascade_tf and self.rig_to_camera_tf and self.rig_to_mountplate_tf and self.mountplate_to_lidar_tf and self.lidar_to_imu_tf:
                    break
                elif transform.header.frame_id == 'base_link' and transform.child_frame_id == 'rig' and not self.base_to_rig_tf:
                    self.base_to_rig_tf = True
                    filename = f'{self.calib_trans_dir_path}/base_to_rig.txt'
                elif transform.header.frame_id == 'rig':
                    if transform.child_frame_id == 'cascade_link' and not self.rig_to_cascade_tf:
                        self.rig_to_cascade_tf = True
                        filename = f'{self.calib_trans_dir_path}/rig_to_cascade.txt'
                    elif transform.child_frame_id == 'mounting_plate' and not self.rig_to_mountplate_tf:
                        self.rig_to_mountplate_tf = True
                        filename = f'{self.calib_trans_dir_path}/rig_to_mountplate.txt'
                    elif transform.child_frame_id == 'camera_link' and not self.rig_to_camera_tf:
                        self.rig_to_camera_tf = True
                        filename = f'{self.calib_trans_dir_path}/rig_to_camera.txt'
                elif transform.header.frame_id == 'mounting_plate' and transform.child_frame_id == 'os_sensor' and not self.mountplate_to_lidar_tf:
                    self.mountplate_to_lidar_tf = True
                    filename = f'{self.calib_trans_dir_path}/mountplate_to_lidar.txt'
                elif transform.header.frame_id == 'os_sensor' and transform.child_frame_id == 'imu_sensor' and not self.lidar_to_imu_tf:
                    self.lidar_to_imu_tf = True
                    filename = f'{self.calib_trans_dir_path}/lidar_to_imu.txt'
    #
                if filename is not None:
                    trans, quat = utils.transform_msg_to_numpy(transform)
                    utils.save_transform(trans, quat, filename)


    def read_bag(self, rosbag_path):
        # Open specified rosbag file
        print('reading bag', rosbag_path)
        bag = rosbag.Bag(rosbag_path)

        for topic, msg, bag_time in bag.read_messages(self.topics_handlers_dict.keys()):
            # print(f"Processing message from topic: {topic}")
            # msg_time = f"{msg.header.stamp.secs}.{msg.header.stamp.nsecs}"
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                msg_header_time = f"{msg.header.stamp.to_sec():.20f}"
            elif hasattr(msg, 'transforms'):
                msg_header_time = None #f"{msg.transforms.header.stamp.to_sec()}"

            # Dispatch message to the corresponding handler function
            self.topics_handlers_dict[topic](msg, msg_header_time)

        # Assign final path attributes after bag has been completely read
        self.fin_path_4x4_ts_data_dict, self.fin_path_quat_ts_data_dict = utils.path_to_numpy(self.path_msg)

        # Close the bag file
        bag.close()


    def write_data_to_files(self):
        # Helper function to handle timestamps, renaming, and saving data
        def save_and_rename(timestamp_file_prefix, ts_index_dict, timestamp_path, data_path, filename_prefix, file_ext='.bin'):
            timestamps_np = np.array(sorted(ts_index_dict.keys()))
            np.savetxt(f'{timestamp_path}/{timestamp_file_prefix}timestamps.txt', timestamps_np, fmt='%s')

            for idx, timestamp in enumerate(timestamps_np):
                original_index = ts_index_dict[timestamp]
                original_filename = f"{data_path}/unsorted_{filename_prefix}_{original_index}{file_ext}"
                new_filename = f"{data_path}/{filename_prefix}_{idx}{file_ext}"
                os.rename(original_filename, new_filename)
        
        # Write and rename CAMERA RGB data
        save_and_rename("rgb_", self.camera_rgb_ts_index_dict, self.camera_path, self.camera_rgb_path, "camera_rgb_image", file_ext='.png')
        
        # Write and rename CAMERA DEPTH data
        save_and_rename("depth_", self.camera_depth_ts_index_dict, self.camera_path, self.camera_depth_path, "camera_depth_image")
        
        # Write and rename LIDAR data
        save_and_rename("", self.ouster_ts_index_dict, self.lidar_path, self.lidar_pc_bin_path, "lidar_pointcloud")

        # Write and rename CASCADE DATACUBE data
        save_and_rename("", self.cascade_datacube_ts_index_dict, self.cascade_datacube_path, self.cascade_datacube_data_path, "frame")

        # Write and rename CASCADE HEATMAP data
        save_and_rename("", self.cascade_heatmap_ts_index_dict, self.cascade_heatmap_path, self.cascade_heatmap_data_path, "heatmap")

        # Write IMU timestamps and data
        imu_timestamps_np = np.array(sorted(self.imu_ts_data_dict.keys()))
        np.savetxt(f'{self.imu_path}/timestamps.txt', imu_timestamps_np, fmt='%s')

        imu_data_np = np.array([self.imu_ts_data_dict[timestamp] for timestamp in imu_timestamps_np])
        np.savetxt(f'{self.imu_path}/imu_data.txt', imu_data_np)

        # Write odometry timestamps and data
        # odom_timestamps_np = np.array(sorted(self.odom_4x4_ts_data_dict.keys()))
        # np.savetxt(f'{self.groundtruth_path}/timestamps.txt', odom_timestamps_np, fmt='%s')

        # odom_4x4_np = np.array([self.odom_4x4_ts_data_dict[timestamp] for timestamp in odom_timestamps_np])
        # np.savetxt(f'{self.groundtruth_path}/groundtruth_poses_4x4.txt', odom_4x4_np)

        # odom_quat_np = np.array([self.odom_quat_ts_data_dict[timestamp] for timestamp in odom_timestamps_np])
        # np.savetxt(f'{self.groundtruth_path}/groundtruth_poses_quat.txt', odom_quat_np) 

        # Write final path timestamps and data to txt files
        fin_path_timestamps_np = np.array(sorted(self.fin_path_4x4_ts_data_dict.keys()))
        np.savetxt(f'{self.groundtruth_path}/timestamps.txt', fin_path_timestamps_np, fmt='%s')

        # fin_path_4x4_np = np.array([self.fin_path_4x4_ts_data_dict[timestamp] for timestamp in fin_path_timestamps_np])
        # np.savetxt(f'{self.groundtruth_path}/groundtruth_final_path_poses_4x4.txt', fin_path_4x4_np)

        fin_path_quat_np = np.array([self.fin_path_quat_ts_data_dict[timestamp] for timestamp in fin_path_timestamps_np])
        np.savetxt(f'{self.groundtruth_path}/groundtruth_poses.txt', fin_path_quat_np) 

        # Save Camera calib info 
        with open(self.camera_path + '/rgb_cam_info.json','w') as f: 
            json.dump(self.rgb_info,f,indent=4) 

        with open(self.camera_path + '/depth_cam_info.json','w') as f:
            json.dump(self.depth_info,f,indent=4)   