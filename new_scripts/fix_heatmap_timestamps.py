import rosbag

# Paths to your original and new ROS bag files
bag_path = './c4c_parking_garage_1_liosam_heatmap.bag'
new_bag_path = './c4c_parking_garage_1_retimed.bag'

# Define the topics to read
data_cube_topic = "/cascade/data_cube"
heatmap_topic = "/cascade/heatmap"
mimo_pcl_topic = "/mimo_pcl"

datacube_msg_ts_bag_ts_dict = {}

# # Open the original bag for reading and the new bag for writing
# with rosbag.Bag(bag_path, 'r') as bag:
#     # Iterate over messages in specified topics
#     for topic, msg, t in bag.read_messages(topics=topics):
#         if topic == data_cube_topic:
#             datacube_msg_ts_bag_ts_dict[f"{msg.header.stamp.secs}.{msg.header.stamp.nsecs}"] = (t.secs, t.nsecs)

with rosbag.Bag(bag_path, 'r') as bag, rosbag.Bag(new_bag_path, 'w') as new_bag:
    for topic, msg, t in bag.read_messages():
        if topic in [data_cube_topic, heatmap_topic, mimo_pcl_topic]:
            print(f"\nReading message from: {topic} with timestamp: {msg.header.stamp.secs}.{msg.header.stamp.nsecs}. BAG timestamp: {t.secs}.{t.nsecs}")
            # new_t_secs, new_t_nsecs = datacube_msg_ts_bag_ts_dict[f"{msg.header.stamp.secs}.{msg.header.stamp.nsecs}"]
            t.secs = int(msg.header.stamp.secs)
            t.nsecs = int(msg.header.stamp.nsecs)
            print(f"New BAG timestamp: {t.secs}.{t.nsecs}")
            # Write the same message to the new bag
            new_bag.write(topic, msg, t)
        else:
            # Write the same message to the new bag
            new_bag.write(topic, msg, t)