#!/usr/bin/env python3

import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import CameraInfo
import json 

def cam_info_to_json(camera_info_msg):
    # Create a dictionary from the CameraInfo message
    camera_info_dict = {
        'header': {
            'seq': camera_info_msg.header.seq,
            'stamp': {
                'secs': camera_info_msg.header.stamp.secs,
                'nsecs': camera_info_msg.header.stamp.nsecs
            },
            'frame_id': camera_info_msg.header.frame_id
        },
        'height': camera_info_msg.height,
        'width': camera_info_msg.width,
        'distortion_model': camera_info_msg.distortion_model,
        'D': camera_info_msg.D,  # Distortion coefficients
        'K': camera_info_msg.K,  # Intrinsic camera matrix
        'R': camera_info_msg.R,  # Rectification matrix
        'P': camera_info_msg.P,  # Projection matrix
        'binning_x': camera_info_msg.binning_x,
        'binning_y': camera_info_msg.binning_y,
        'roi': {
            'x_offset': camera_info_msg.roi.x_offset,
            'y_offset': camera_info_msg.roi.y_offset,
            'height': camera_info_msg.roi.height,
            'width': camera_info_msg.roi.width,
            'do_rectify': camera_info_msg.roi.do_rectify
        }
    }

    # Convert dictionary to JSON string
    camera_info_json = json.dumps(camera_info_dict, indent=4)
    return camera_info_json


def pointcloud_msg_to_numpy(msg, datatype=np.float32):
    # Decode the point cloud
    field_names = ['x', 'y', 'z', 'intensity', 'reflectivity']
    points = pc2.read_points(msg, field_names=field_names, skip_nans=True)
    pointcloud_numpy = np.array(list(points), dtype=datatype)

    return pointcloud_numpy

def image_msg_to_numpy(msg, datatype=np.uint8, num_channels=3):
    '''
    Return ROS-based image data to numpy array.

    '''
    # # Decode the rgb image: Convert the image data to a numpy array and reshape it.
    # if msg.encoding != 'rgb8': # RGB image
    #     raise ValueError(f"RGB encoding incorrect! Expected rgb8, got {msg.encoding}.")
    # if msg.encoding != '16UC1': # Monochrome depth image
    #     raise ValueError(f"Depth image encoding incorrect! Expected 16UC1, got {msg.encoding}.")
    
    image_numpy = np.frombuffer(msg.data, dtype=datatype).reshape(msg.height, msg.width, num_channels)
    
    return image_numpy


def transform_msg_to_numpy(msg, offset=None):
    '''
    
    '''
    translation = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
    quaternion = np.array([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])

    if offset is not None:
        off_t, off_q = offset
        t += off_t
        w = quaternion[3] * off_q[3] - np.dot(quaternion[:3], off_q[:3])
        v = quaternion[3] * off_q[:3] + off_q[3] * quaternion[:3] + np.cross(quaternion[:3], off_q[:3])
        quaternion[3] = w
        quaternion[:3] = v

    return translation, quaternion

def path_to_numpy(msg): 
    # Preallocate the ndarray with shape (N, 8), where N is the number of poses
    path_array = np.zeros((len(msg.poses), 8))

    # Loop over each pose in the path message
    for i, pose_stamped in enumerate(msg.poses):
        # Extract timestamp in nanoseconds
        timestamp_ns = pose_stamped.header.stamp.to_nsec()

        # Extract position (x, y, z)
        position = pose_stamped.pose.position
        x, y, z = position.x, position.y, position.z

        # Extract orientation quaternion (qx, qy, qz, qw)
        orientation = pose_stamped.pose.orientation
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w

        # Store in the ndarray (timestamp, x, y, z, qx, qy, qz, qw)
        path_array[i] = [timestamp_ns, x, y, z, qx, qy, qz, qw]

    return path_array

def pose_msg_to_4x4_numpy(msg):
    '''
    
    '''
    odom_4x4_numpy = np.zeros((4, 4))
    odom_4x4_numpy[0, 3] = msg.pose.pose.position.x
    odom_4x4_numpy[1, 3] = msg.pose.pose.position.y
    odom_4x4_numpy[2, 3] = msg.pose.pose.position.z
    odom_4x4_numpy[3, 3] = 1

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
    odom_4x4_numpy[:3, :3] = R
            
    return odom_4x4_numpy


def odometry_msg_to_numpy(msg):
    '''
    Returns flattened pose messages with 4x4 style and quaternion style.

    '''

    odom_4x4_flat_numpy = pose_msg_to_4x4_numpy(msg).flatten()
    odom_quat_flat_numpy = np.asarray([msg.pose.pose.position.x, 
                                       msg.pose.pose.position.y, 
                                       msg.pose.pose.position.z,
                                       msg.pose.pose.orientation.x,
                                       msg.pose.pose.orientation.y,
                                       msg.pose.pose.orientation.z,
                                       msg.pose.pose.orientation.w])
    
    return odom_4x4_flat_numpy, odom_quat_flat_numpy


def imu_msg_to_numpy(msg):
    imu_numpy = np.asarray([msg.linear_acceleration.x,
                            msg.linear_acceleration.y,
                            msg.linear_acceleration.z,
                            msg.angular_velocity.x,
                            msg.angular_velocity.y,
                            msg.angular_velocity.z,
                            msg.orientation.x,
                            msg.orientation.y,
                            msg.orientation.z,
                            msg.orientation.w])
    
    return imu_numpy