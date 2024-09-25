#!/usr/bin/env python3
"""
ColoRadar Processing Tools - crpt_rosbag_parser.py

Author: Doncey Albin
"""

import os
import numpy as np
np.set_printoptions(suppress=True)
from array import array
import rosbag
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import CameraInfo

def write_transform(msg, filename, offset=None):
  t = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
  q = np.array([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
  if offset is not None:
    off_t, off_q = offset
    t += off_t
    w = q[3] * off_q[3] - np.dot(q[:3],off_q[:3])
    v = q[3] * off_q[:3] + off_q[3] * q[:3] + np.cross(q[:3],off_q[:3])
    q[3] = w
    q[:3] = v
  file = open(filename, 'w')
  file.write(str(t[0]) + ' ' + str(t[1]) + ' ' + str(t[2]) + '\n')
  file.write(str(q[0]) + ' ' + str(q[1]) + ' ' + str(q[2]) + ' ' + str(q[3]) + '\n')
  file.close()


class BagParser:
    def __init__(self, visualize_rosbag_data, log_paths_dict, visulizer = None):
        self.visualize_rosbag_data = visualize_rosbag_data
        self.visualizer = visulizer
        
        # Set paths for config
        self.calib_trans_dir_path = log_paths_dict['calib_trans_dir_path']
        self.calib_cascade_dir_path = log_paths_dict['calib_cascade_dir_path']
        
        # Set paths for run
        self.lidar_path = log_paths_dict['lidar_path']
        self.lidar_pc_bin_path = log_paths_dict['lidar_pc_bin_path']
        self.camera_path = log_paths_dict['camera_path']
        self.camera_rgb_path = log_paths_dict['camera_rgb_path']
        self.camera_depth_path = log_paths_dict['camera_depth_path']
        self.groundtruth_path = log_paths_dict['groundtruth_path']
        self.imu_path = log_paths_dict['imu_path']
        self.cascade_datacube_path = log_paths_dict['cascade_datacube_path']
        self.cascade_datacube_data_path = log_paths_dict['cascade_datacube_data_path']
        self.cascade_heatmap_path = log_paths_dict['cascade_heatmap_path']
        self.cascade_heatmap_data_path = log_paths_dict['cascade_heatmap_data_path']

        # Set topics
        self.ouster_points_topic = "/ouster/points"
        self.imu_data_topic = "/gx5/imu/data"
        self.odom_topic = "/lio_sam/mapping/odometry"
        self.camera_depth_image_topic = '/camera/depth/image_rect_raw'
        self.camera_depth_info_topic = '/camera/depth/camera_info'
        self.camera_rgb_image_topic = "/camera/color/image_raw"
        self.camera_rgb_info_topic = '/camera/color/camera_info'
        self.cascade_datacube_topic = "/cascade/data_cube"
        self.cascade_heatmap_topic = "/cascade/heatmap"
        # self.transforms_topic = "/tf"
        self.static_transforms_topic = "/tf_static"

        # Dictionary that maps topics to their respective handling functions. Keys can be used as wanted topics.
        self.topics_handlers_dict = {
            self.ouster_points_topic: self.handle_ouster_pointcloud,
            self.imu_data_topic: self.handle_imu_data,
            self.camera_rgb_image_topic: self.handle_rgb_image,
            self.camera_depth_image_topic: self.handle_depth_image,
            self.odom_topic: self.handle_odometry,
            self.cascade_datacube_topic: self.handle_cascade_datacube,
            self.cascade_heatmap_topic: self.handle_cascade_heatmap,
            # self.transforms_topic: self.handle_transforms,
            self.static_transforms_topic: self.handle_transforms,
        }

        # Initialize number of data to 1 so that it corresponds with line in timestamp text file
        self.lidar_pc_num = 1
        self.camera_rgb_num = 1
        self.camera_depth_num = 1
        self.cascade_heatmap_num = 1
        self.cascade_datacube_num = 1

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
        self.camera_depth_ts_index_dict = {}   
        self.camera_rgb_ts_index_dict = {}
        self.cascade_heatmap_ts_index_dict = {}
        self.cascade_datacube_ts_index_dict = {}


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
        # lidar_t_file.write('%6f\n' % (msg.header.stamp.to_sec()))
        self.odom_4x4_ts_data_dict[msg_time] = odom_4x4_flat
        self.odom_quat_ts_data_dict[msg_time] = odom_quat
        # gt_data_file.write(str(p_x) + ' ' + str(p_y) + ' ' + str(p_z) + ' ' + str(q_x) + ' ' + str(q_y) + ' ' + str(q_z) + ' ' + str(q_w) + '\n')


    def handle_imu_data(self, msg, msg_time):
        # Decode IMU
        self.imu_ts_data_dict[msg_time] = np.asarray([msg.linear_acceleration.x,
                                                      msg.linear_acceleration.y,
                                                      msg.linear_acceleration.z,
                                                      msg.angular_velocity.x,
                                                      msg.angular_velocity.y,
                                                      msg.angular_velocity.z,])
                                                    #   msg.orientation.x,
                                                    #   msg.orientation.y,
                                                    #   msg.orientation.z,
                                                    #   msg.orientation.w])


    def gen_cascade_heatmap_config(self, msg):
        cascade_heatmap_config_file_path = os.path.join(self.calib_cascade_dir_path, 'heatmap_cfg.txt')
        if not os.path.exists(cascade_heatmap_config_file_path):
            cascade_heatmap_config_file = open(cascade_heatmap_config_file_path, 'w')
            cascade_heatmap_config_file.write('num_range_bins ' + str(msg.depth) + '\n')
            cascade_heatmap_config_file.write('num_elevation_bins ' + str(msg.height) + '\n')
            cascade_heatmap_config_file.write('num_azimuth_bins ' + str(msg.width) + '\n')
            #cascade_hm_file.write('num_doppler_bins ' + str(msg.num_doppler_bins) + '\n')
            cascade_heatmap_config_file.write('range_bin_width ' + str(msg.range_bin_width) + '\n')
            #cascade_hm_file.write('doppler_bin_width ' + str(msg.doppler_bin_width) + '\n')
            cascade_heatmap_config_file.write('azimuth_bins')
            for i in range(msg.width):
                cascade_heatmap_config_file.write(' ' + str(msg.azimuth_bins[i]))
                cascade_heatmap_config_file.write('\n')
                cascade_heatmap_config_file.write('elevation_bins')
            for i in range(msg.height):
                cascade_heatmap_config_file.write(' ' + str(msg.elevation_bins[i]))
                cascade_heatmap_config_file.write('\n')
            cascade_heatmap_config_file.close()

    def handle_cascade_heatmap(self, msg, msg_time):
        # Generate config file if not yet exists
        self.gen_cascade_heatmap_config(msg)

        # Decode the heatmap
        cascade_heatmap_array = np.array(msg.image, dtype=np.float32)
        # Save cascade heatmap
        cascade_heatmap_filename = f"{self.cascade_heatmap_data_path}/unsorted_heatmap_{self.cascade_heatmap_num}.bin"
        cascade_heatmap_array.tofile(cascade_heatmap_filename)
        self.cascade_heatmap_ts_index_dict[msg_time] = self.cascade_heatmap_num
        self.cascade_heatmap_num += 1


    def gen_cascade_datacube_config(self, msg):
        cascade_datacube_config_file_path = os.path.join(self.calib_cascade_dir_path, 'waveform_cfg.txt')
        if not os.path.exists(cascade_datacube_config_file_path):
            cascade_datacube_config_file = open(cascade_datacube_config_file_path, 'w')
            cascade_datacube_config_file.write('num_rx ' + str(msg.num_rx) + '\n')
            cascade_datacube_config_file.write('num_tx ' + str(msg.num_tx) + '\n')
            cascade_datacube_config_file.write('num_adc_samples_per_chirp ' + str(msg.num_adc_samples_per_chirp) + '\n')
            cascade_datacube_config_file.write('num_chirps_per_frame ' + str(msg.num_chirps_per_frame) + '\n')
            cascade_datacube_config_file.write('adc_sample_frequency ' + str(msg.adc_sample_frequency) + '\n')
            cascade_datacube_config_file.write('start_frequency ' + str(msg.start_frequency) + '\n')
            cascade_datacube_config_file.write('idle_time ' + str(msg.idle_time) + '\n')
            cascade_datacube_config_file.write('adc_start_time ' + str(msg.adc_start_time) + '\n')
            cascade_datacube_config_file.write('ramp_end_time ' + str(msg.ramp_end_time) + '\n')
            cascade_datacube_config_file.write('frequency_slope ' + str(msg.frequency_slope) + '\n')
            cascade_datacube_config_file.close()

    def handle_cascade_datacube(self, msg, msg_time):
        # Generate config file if not yet exists
        self.gen_cascade_datacube_config(msg)

        # Decode the datacube
        cascade_datacube_array = np.array(msg.samples, dtype=np.int16)
        # Save cascade datacube
        cascade_datacube_filename = f"{self.cascade_datacube_data_path}/unsorted_frame_{self.cascade_datacube_num}.bin"
        cascade_datacube_array.tofile(cascade_datacube_filename)
        self.cascade_datacube_ts_index_dict[msg_time] = self.cascade_datacube_num
        self.cascade_datacube_num += 1

    def handle_transforms(self, msg, msg_time):
        if hasattr(msg, 'transforms'):
            for transform in msg.transforms:
                if transform.header.frame_id == 'base_link' and transform.child_frame_id == 'rig' and not self.base_to_rig_tf:
                    self.base_to_rig_tf = True
                    write_transform(transform, f'{self.calib_trans_dir_path}/base_to_rig.txt')
                if transform.header.frame_id == 'rig':
                    if transform.child_frame_id == 'cascade_link' and not self.rig_to_cascade_tf:
                        self.rig_to_cascade_tf = True
                        write_transform(transform, f'{self.calib_trans_dir_path}/rig_to_cascade.txt')
                    elif transform.child_frame_id == 'mounting_plate' and not self.rig_to_mountplate_tf:
                        self.rig_to_mountplate_tf = True
                        write_transform(transform, f'{self.calib_trans_dir_path}/rig_to_mountplate.txt')
                    elif transform.child_frame_id == 'camera_link' and not self.rig_to_camera_tf:
                        self.rig_to_camera_tf = True
                        write_transform(transform, f'{self.calib_trans_dir_path}/rig_to_camera.txt')
                if transform.header.frame_id == 'mounting_plate' and transform.child_frame_id == 'os_sensor' and not self.mountplate_to_lidar_tf:                      
                    self.mountplate_to_lidar_tf = True
                    write_transform(transform, f'{self.calib_trans_dir_path}/mountplate_to_lidar.txt')
                if transform.header.frame_id == 'os_sensor' and transform.child_frame_id == 'imu_sensor' and not self.lidar_to_imu_tf: 
                    self.lidar_to_imu_tf = True
                    write_transform(transform, f'{self.calib_trans_dir_path}/lidar_to_imu.txt')

                if self.base_to_rig_tf and self.rig_to_cascade_tf and self.rig_to_camera_tf and self.rig_to_mountplate_tf and self.mountplate_to_lidar_tf and self.lidar_to_imu_tf:
                    break


    def read_bag(self, rosbag_path):
        # Open specified rosbag file
        bag = rosbag.Bag(rosbag_path)

        for topic, msg, bag_time in bag.read_messages(self.topics_handlers_dict.keys()):
            # print(f"Processing message from topic: {topic}")
            # msg_time = f"{msg.header.stamp.secs}.{msg.header.stamp.nsecs}"
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                msg_time = f"{msg.header.stamp.to_sec():.6f}"
            elif hasattr(msg, 'transforms'):
                msg_time = None #f"{msg.transforms.header.stamp.to_sec()}"

            # Dispatch message to the corresponding handler function
            self.topics_handlers_dict[topic](msg, msg_time)

        # Close the bag file
        bag.close()


    def write_data_to_files(self):
        # Helper function to handle timestamps, renaming, and saving data
        def save_and_rename(timestamp_file_prefix, ts_index_dict, timestamp_path, data_path, filename_prefix):
            timestamps_np = np.array(sorted(ts_index_dict.keys()))
            np.savetxt(f'{timestamp_path}/{timestamp_file_prefix}timestamps.txt', timestamps_np, fmt='%s')

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
        odom_timestamps_np = np.array(sorted(self.odom_4x4_ts_data_dict.keys()))
        np.savetxt(f'{self.groundtruth_path}/timestamps.txt', odom_timestamps_np, fmt='%s')

        odom_4x4_np = np.array([self.odom_4x4_ts_data_dict[timestamp] for timestamp in odom_timestamps_np])
        np.savetxt(f'{self.groundtruth_path}/groundtruth_poses_4x4.txt', odom_4x4_np)

        odom_quat_np = np.array([self.odom_quat_ts_data_dict[timestamp] for timestamp in odom_timestamps_np])
        np.savetxt(f'{self.groundtruth_path}/groundtruth_poses_quat.txt', odom_quat_np)