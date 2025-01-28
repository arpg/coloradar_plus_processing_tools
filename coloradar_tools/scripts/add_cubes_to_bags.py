import os
import sys
import rosbag
import pandas as pd
import argparse


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


from std_msgs.msg import String  # Replace with your actual message type for topic2

def processing_function(data):
    """
    A placeholder for your data processing function.
    Modify this function based on the processing logic you need.
    """
    # Example: Convert the input data to uppercase (for String messages)
    return data.upper()

def clone_topic_with_processing(input_bag_path, output_bag_path, topic1, topic2, msg_type):
    """
    Process all messages in topic1 and save the result into topic2.

    Args:
        input_bag_path (str): Path to the input bag file.
        output_bag_path (str): Path to the output bag file.
        topic1 (str): The source topic name.
        topic2 (str): The target topic name.
        msg_type: The ROS message type for the target topic.
    """
    try:
        with rosbag.Bag(input_bag_path, 'r') as input_bag, rosbag.Bag(output_bag_path, 'w') as output_bag:
            for topic, msg, t in input_bag.read_messages(topics=[topic1]):
                if hasattr(msg, 'header'):
                    # Clone the original message header
                    header = msg.header
                else:
                    raise ValueError(f"Message in topic '{topic1}' has no header. Cannot clone header.")

                # Process the data field
                processed_data = processing_function(msg.data)

                # Create a new message for topic2
                new_msg = msg_type()
                new_msg.header = header  # Clone the header
                new_msg.data = processed_data  # Set the processed data

                # Write the new message to topic2 in the output bag
                output_bag.write(topic2, new_msg, t)
                print(f"Processed and saved message at time {t.to_sec()}...", end="\r")

        print(f"\nProcessing completed. Saved new messages to topic '{topic2}' in '{output_bag_path}'.")

    except Exception as e:
        print(f"Error: {e}")


# Example usage
if __name__ == "__main__":
    input_bag = "path/to/large_input.bag"
    output_bag = "path/to/output.bag"
    topic1 = "/topic1"
    topic2 = "/topic2"

    # Replace String with the actual message type for topic2
    clone_topic_with_processing(input_bag, output_bag, topic1, topic2, String)


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


import struct


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


def add_heatmap_to_bag(bag, heatmap):
    pass


def read_bag(bag_path):
    pass


def run():
    bags_directory = ''

    COLORADAR_PATH = os.path.join(os.path.expanduser('~'), 'coloradar')
    dataset = tools.ColoradarDataset(COLORADAR_PATH)
    processor = cuda_tools.RadarProcessor(config=dataset.cascade_config())

    for filename in os.listdir(bags_directory):
        if filename.endswith(".bag"):
            bag_path = os.path.join(bags_directory, filename)
            for cube in read_bag(bag_path):
                # copy header and frame id for all data
                hm = processor.cubeToHeatmap(cube)
                add_heatmap_to_bag(bag_path, hm)


if __name__ == "__main__":
    # parser = argparse.ArgumentParser(description="Get information about all topics in a bag file.")
    # parser.add_argument("bag_file", help="Path to the ROS bag file.")
    # args = parser.parse_args()
    # file_name = '/media/arpg/brendan-ssd/coloradarplus_data/bike_path_run0_liosam.bag'
    if in_docker:
        file_name = '/root/bags/bike_path_run0_liosam.bag'
    else:
        file_name = None

    get_topic_info(file_name, '/cascade/data_cube')

    # parser = argparse.ArgumentParser(description="Get information about a specific topic in a bag file.")
    # parser.add_argument("bag_file", help="Path to the ROS bag file.")
    # parser.add_argument("topic_name", help="Name of the topic to analyze.")
    #
    # args = parser.parse_args()
    #
    # get_topic_info(args.bag_file, args.topic_name)
