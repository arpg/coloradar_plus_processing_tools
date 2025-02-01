import os
import sys
import rosbag
import pandas as pd
import struct
from tqdm import tqdm
from collections import namedtuple


in_docker = os.path.exists("/.dockerenv")
if in_docker:
    build_dir = '/src/coloradar_tools/build'
else:
    cwd = os.getcwd()
    if cwd.endswith(os.path.join("coloradar_plus_processing_tools", "coloradar_tools")):
        build_dir = os.path.join(cwd, "..", "build")
    else:
        build_dir = os.path.join(cwd, "build")
sys.path.append(build_dir)
import coloradar_dataset_tools as tools
import coloradar_cuda_tools as cuda_tools



HeatmapMsg = namedtuple("HeatmapMsg", [
    "header", "depth", "height", "width", "num_doppler_bins",
    "range_bin_width", "doppler_bin_width", "azimuth_bins",
    "elevation_bins", "image"
])


def add_heatmaps(radar_processor, bag_path, cube_topic='/cascade/data_cube', heatmap_topic='/cascade/heatmap'):
    with rosbag.Bag(bag_path, 'r') as bag:
        if heatmap_topic in bag.get_type_and_topic_info().topics:
            print(f"Skipping {bag_path}, heatmaps already exist.")
            return

    with rosbag.Bag(bag_path, 'a') as bag:
        for topic, cube_msg, t in tqdm(bag.read_messages(topics=[cube_topic])):
            heatmap_msg = HeatmapMsg(
                header=cube_msg.header,
                depth=radar_processor.config.num_pos_range_bins,
                height=radar_processor.config.num_elevation_beams,
                width=radar_processor.config.num_azimuth_beams,
                num_doppler_bins=radar_processor.config.num_doppler_bins,
                range_bin_width=radar_processor.config.range_bin_width,
                doppler_bin_width=radar_processor.config.doppler_bin_width,
                azimuth_bins=radar_processor.config.azimuth_bins,
                elevation_bins=radar_processor.config.elevation_bins,
                image=radar_processor.cube_to_heatmap(cube_msg.samples)
            )
            bag.write(heatmap_topic, heatmap_msg, t)


def get_topic_info(bag_file, topic_name):
    try:
        # Open the bag file
        bag = rosbag.Bag(bag_file)

        # Initialize counters and variables
        message_count = 0
        message_type = None
        header_fields = set()

        # Iterate through messages in the specified topic
        for topic, msg, _ in bag.read_messages(topics=[topic_name]):
            message_count += 1
            if message_type is None:
                message_type = type(msg).__name__  # Get the message type

            # Extract header fields if the message has a header
            if hasattr(msg, 'header'):
                header_fields.update(msg.header.__slots__)

        # Print topic information
        print(f"Topic: {topic_name}")
        print(f"Number of messages: {message_count}")
        print(f"Message type: {message_type}")
        if header_fields:
            print(f"Header fields: {', '.join(header_fields)}")
        else:
            print("No header fields found.")

        # Close the bag file
        bag.close()

    except Exception as e:
        print(f"Error: {e}")


def get_all_topics_info(bag_file, chunk_size=1024**3):
    with open(bag_file, 'rb') as f:
        topics = []
        message_counts = {}
        message_types = {}

        while True:
            # Read a chunk of the file
            chunk = f.read(chunk_size)
            if not chunk:
                break  # End of file reached

            # Process the chunk
            offset = 0
            while offset < len(chunk):
                # Read the topic length (4 bytes)
                if offset + 4 > len(chunk):
                    break  # Prevent partial read at the end of the chunk
                topic_length = struct.unpack('I', chunk[offset:offset + 4])[0]
                offset += 4

                # Read the topic name
                if offset + topic_length > len(chunk):
                    break
                topic_name = chunk[offset:offset + topic_length].decode('utf-8')
                offset += topic_length

                # Read the message type length (4 bytes)
                if offset + 4 > len(chunk):
                    break
                message_type_length = struct.unpack('I', chunk[offset:offset + 4])[0]
                offset += 4

                # Read the message type
                if offset + message_type_length > len(chunk):
                    break
                message_type = chunk[offset:offset + message_type_length].decode('utf-8')
                offset += message_type_length

                # Read the message count (4 bytes)
                if offset + 4 > len(chunk):
                    break
                message_count = struct.unpack('I', chunk[offset:offset + 4])[0]
                offset += 4

                if topic_name not in message_counts:
                    message_counts[topic_name] = 0
                    message_types[topic_name] = message_type

                message_counts[topic_name] += message_count
                topics.append({
                    'topic': topic_name,
                    'message_type': message_type,
                    'message_count': message_count
                })

        # Convert to DataFrame
        df = pd.DataFrame(topics)

        # Print topic information
        for topic_name, count in message_counts.items():
            print(f"\nTopic: {topic_name}")
            print(f"Number of messages: {count}")
            print(f"Message type: {message_types[topic_name]}")

        # Print DataFrame
        print("\nMessage Data:")
        print(df.head())


def run():
    bags_directory = '/root/bags'
    dataset_path = os.path.join(bags_directory, 'bin_dataset')
    dataset = tools.ColoradarPlusDataset(dataset_path)
    processor = cuda_tools.RadarProcessor(config=dataset.cascade_config())

    for filename in os.listdir(bags_directory):
        if filename.endswith(".bag"):
            bag_path = os.path.join(bags_directory, filename)
            print(f'Processing bag {bag_path}')
            add_heatmaps(processor, bag_path)


if __name__ == "__main__":
    run()
    # parser = argparse.ArgumentParser(description="Get information about all topics in a bag file.")
    # parser.add_argument("bag_file", help="Path to the ROS bag file.")
    # args = parser.parse_args()
    # file_name = '/media/arpg/brendan-ssd/coloradarplus_data/bike_path_run0_liosam.bag'
    # if in_docker:
    #     file_name = '/root/bags/bike_path_run0_liosam.bag'
    # else:
    #     file_name = None

    # get_topic_info(file_name, '/cascade/data_cube')

    # parser = argparse.ArgumentParser(description="Get information about a specific topic in a bag file.")
    # parser.add_argument("bag_file", help="Path to the ROS bag file.")
    # parser.add_argument("topic_name", help="Name of the topic to analyze.")
    #
    # args = parser.parse_args()
    #
    # get_topic_info(args.bag_file, args.topic_name)
