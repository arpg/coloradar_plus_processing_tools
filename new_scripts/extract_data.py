import rosbag
import numpy as np
import open3d as o3d
import struct
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2

def decode_pointcloud2(cloud, field_names, display_points=False):
    points = pc2.read_points(cloud, field_names=field_names, skip_nans=True)
    points_array = np.array(list(points))
    
    if display_points:
        visualize_pointcloud(points_array)

    return points_array

def visualize_pointcloud(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])
    
    intensity = points[:, 3]
    intensity_normalized = (intensity - np.min(intensity)) / (np.max(intensity) - np.min(intensity))
    
    colors = np.tile(intensity_normalized[:, None], (1, 3))
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    o3d.visualization.draw_geometries([pcd])

def decode_realsense_image(msg, display_image=False):
    """
    Decode a ROS sensor_msgs/Image message into a numpy array.
    """
    if msg.encoding == '16UC1': # Monochrome depth image
        dtype = np.uint16
        channels = 1
    elif msg.encoding == 'rgb8': # RGB image
        dtype = np.uint8
        channels = 3
    else:
        raise ValueError(f"Unsupported encoding: {msg.encoding}")
    
    # Convert the image data to a numpy array and reshape it
    image = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)

    if display_image:
        plt.imshow(image,cmap='gray')
        plt.title('Depth Image')
        plt.axis('off')
        plt.show()
    return image

def decode_odom(msg):
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
    return odom_mat

# Path to your bag file
bag_path = './ec_courtyard_1_liosam.bag'

# Open the bag file
bag = rosbag.Bag(bag_path)

ouster_points_topic = "/ouster/points"
field_names = ['x', 'y', 'z', 'intensity', 'reflectivity']

odom_topic = "/lio_sam/mapping/odometry"

depth_image_topic = '/camera/depth/image_rect_raw'
rs_depth_scan_num = 1
rs_depth_timestamp_list = []
rgb_image_topic = "/camera/color/image_raw"
rs_rgb_scan_num = 1
rs_rgb_timestamp_list = []

# Dictionaries for timestamps and data
ouster_timestamps = []
odom_data_dict = {}

# Read messages
for topic, msg, t in bag.read_messages(topics=[ouster_points_topic, odom_topic]):
    print(f"Processing message from topic: {topic}")
    msg_time = f"{msg.header.stamp.secs}.{msg.header.stamp.nsecs}"
    
    if topic == ouster_points_topic:
        # Decode the point cloud
        pointcloud = decode_pointcloud2(msg, field_names, display_points=False)
        pointcloud = np.array(list(pointcloud), dtype=np.float32)
        print(pointcloud.shape)
        filename = f"./data/ouster/data/{msg_time.replace('.', '_')}.bin"
        pointcloud.tofile(filename)
        ouster_timestamps.append(msg_time)
    # if topic in [rgb_image_topic, depth_image_topic]:
    #     # Decode the image message to a numpy array
    #     image = decode_realsense_image(msg, display_image=False)
    #     if topic == rgb_image_topic:
    #         #  image.tofile(f"./data/realsense/rgb_images/data/{rs_rgb_scan_num:05d}.bin")
    #         rs_rgb_scan_num += 1
    #         rs_rgb_timestamp_list.append(msg_time)
    #     elif topic == depth_image_topic:
    #         #  image.tofile(f"./data/realsense/depth_images/data/{rs_depth_scan_num:05d}.bin")
    #         rs_depth_scan_num += 1
    #         rs_depth_timestamp_list.append(msg_time)
    if topic == odom_topic:
        # Decode the odometry
        odom_mat = decode_odom(msg)
        odom_flat = odom_mat.flatten()
        odom_data_dict[msg_time] = odom_flat

# Extract sorted timestamps
sorted_ouster_timestamps = sorted(ouster_timestamps)
sorted_odom_timestamps = sorted(odom_data_dict.keys())

# # Write Realsense timestamps and data
# rs_rgb_timestamps_np = sorted(rs_rgb_timestamp_list)
# rs_depth_timestamps_np = sorted(rs_depth_timestamp_list)
# np.savetxt('data/realsense/rgb_images/timestamps.txt', rs_rgb_timestamps_np, fmt='%s')
# np.savetxt('data/realsense/depth_images/timestamps.txt', rs_depth_timestamps_np, fmt='%s')

# Write ouster timestamps and data
ouster_timestamps_np = np.array(sorted_ouster_timestamps)
np.savetxt('data/ouster/timestamps.txt', ouster_timestamps_np, fmt='%s')

# Write odometry timestamps and data
odom_timestamps_np = np.array(sorted_odom_timestamps)
np.savetxt('data/poses/timestamps.txt', odom_timestamps_np, fmt='%s')

sorted_odom_list = [odom_data_dict[timestamp] for timestamp in sorted_odom_timestamps]
odom_np = np.array(sorted_odom_list)
np.savetxt('data/poses/odom.txt', odom_np)

# Close the bag file
bag.close()

print("Data processing complete.")
