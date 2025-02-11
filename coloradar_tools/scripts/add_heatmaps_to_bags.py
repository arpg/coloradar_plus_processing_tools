import os
import sys
import rosbag
import pandas as pd
import numpy as np
import struct
from tqdm import tqdm
import struct
from dca1000_device.msg import MimoMsg


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


# HeatmapMsg = namedtuple("HeatmapMsg", [
#     "header", "depth", "height", "width", "num_doppler_bins",
#     "range_bin_width", "doppler_bin_width", "azimuth_bins",
#     "elevation_bins", "image"
# ])


# class HeatmapMsg:
#     _md5sum = "7905ef9ff8d4d3408ed1b3f399d5f46f"
#     _type = "dca1000_device/MimoMsg"
#     _has_header = True
#     _full_text = """\
# std_msgs/Header header
# uint8 depth
# uint8 height
# uint8 width
# uint8 num_doppler_bins
# float32 range_bin_width
# float32 doppler_bin_width
# float32[] azimuth_bins
# float32[] elevation_bins
# float32[] image
# """
#     _slot_types = [
#         "std_msgs/Header", "uint8", "uint8", "uint8", "uint8",
#         "float32", "float32", "float32[]", "float32[]", "float32[]"
#     ]
#
#     __slots__ = [
#         "header", "depth", "height", "width", "num_doppler_bins",
#         "range_bin_width", "doppler_bin_width", "azimuth_bins",
#         "elevation_bins", "image"
#     ]
#
#     def __init__(self, header=None, depth=0, height=0, width=0, num_doppler_bins=0,
#                  range_bin_width=0.0, doppler_bin_width=0.0, azimuth_bins=None,
#                  elevation_bins=None, image=None):
#         # Enforce correct data types
#         self.header = header if header is not None else self._default_header()
#         self.depth = np.uint8(depth)
#         self.height = np.uint8(height)
#         self.width = np.uint8(width)
#         self.num_doppler_bins = np.uint8(num_doppler_bins)
#         self.range_bin_width = np.float32(range_bin_width)
#         self.doppler_bin_width = np.float32(doppler_bin_width)
#
#         # Ensure array attributes are float32 NumPy arrays
#         self.azimuth_bins = np.array(azimuth_bins, dtype=np.float32) if azimuth_bins is not None else np.array([], dtype=np.float32)
#         self.elevation_bins = np.array(elevation_bins, dtype=np.float32) if elevation_bins is not None else np.array([], dtype=np.float32)
#         self.image = np.array(image, dtype=np.float32) if image is not None else np.array([], dtype=np.float32)
#
#     @staticmethod
#     def _default_header():
#         """Generate a default std_msgs/Header-like object."""
#         class Header:
#             __slots__ = ["seq", "stamp", "frame_id"]
#
#             def __init__(self):
#                 self.seq = np.uint32(0)
#                 self.stamp = {"secs": np.uint32(0), "nsecs": np.uint32(0)}  # Mimic rospy.Time
#                 self.frame_id = ""
#
#         return Header()
#
#     def serialize(self, buff):
#         """Serialize the message into a binary buffer."""
#         # Pack header
#         buff.write(struct.pack("<I", self.header.seq))
#         buff.write(struct.pack("<II", self.header.stamp["secs"], self.header.stamp["nsecs"]))
#         buff.write(struct.pack("<I", len(self.header.frame_id)) + self.header.frame_id.encode())
#
#         # Pack scalar fields
#         buff.write(struct.pack("<BBBBff", self.depth, self.height, self.width, self.num_doppler_bins,
#                                self.range_bin_width, self.doppler_bin_width))
#
#         # Pack array fields
#         for arr in [self.azimuth_bins, self.elevation_bins, self.image]:
#             buff.write(struct.pack("<I", len(arr)))  # Store array length
#             buff.write(struct.pack("<{}f".format(len(arr)), *arr))  # Store array data
#
#     def deserialize(self, buff):
#         """Deserialize the message from a binary buffer."""
#         # Read header
#         self.header.seq = struct.unpack("<I", buff.read(4))[0]
#         secs, nsecs = struct.unpack("<II", buff.read(8))
#         self.header.stamp = {"secs": secs, "nsecs": nsecs}
#         frame_id_len = struct.unpack("<I", buff.read(4))[0]
#         self.header.frame_id = buff.read(frame_id_len).decode()
#
#         # Read scalar fields
#         (self.depth, self.height, self.width, self.num_doppler_bins,
#          self.range_bin_width, self.doppler_bin_width) = struct.unpack("<BBBBff", buff.read(10))
#
#         # Read array fields
#         self.azimuth_bins = self._read_float_array(buff)
#         self.elevation_bins = self._read_float_array(buff)
#         self.image = self._read_float_array(buff)
#
#     def _read_float_array(self, buff):
#         """Helper function to deserialize float arrays."""
#         arr_len = struct.unpack("<I", buff.read(4))[0]
#         if arr_len == 0:
#             return np.array([], dtype=np.float32)
#         return np.array(struct.unpack("<{}f".format(arr_len), buff.read(4 * arr_len)), dtype=np.float32)



def add_heatmaps(radar_processor, bag_path, cube_topic='/cascade/data_cube', heatmap_topic='/cascade/heatmap'):
    with rosbag.Bag(bag_path, 'r') as bag:
        if heatmap_topic in bag.get_type_and_topic_info().topics:
            print(f"Skipping {bag_path}, heatmaps already exist.")
            return

    with rosbag.Bag(bag_path, 'a') as bag:
        for topic, cube_msg, t in tqdm(bag.read_messages(topics=[cube_topic])):
            heatmap_msg = MimoMsg(
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
