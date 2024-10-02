#!/usr/bin/env python3

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
import open3d as o3d
import os
import rosbag
import re

from coloradar_plus_processing_tools import utils

def get_bag_duration(bag_path):
    bag = rosbag.Bag(bag_path)
    start_time = bag.get_start_time()
    end_time = bag.get_end_time()
    duration = end_time - start_time
    bag.close()
    return duration
    
def get_bag_path_length(bag_path, odom_topic):
    # Open the bag file
    odom_list = []
    bag = rosbag.Bag(bag_path)
    for topic, msg, t in bag.read_messages(topics=odom_topic):
        odom_4x4_flat_numpy, odom_quat_flat_numpy = utils.odometry_msg_to_numpy(msg)
        odom_list.append(odom_4x4_flat_numpy)

    # Extract positions from the odometry data
    positions = [(odom[3], odom[7], odom[11]) for odom in odom_list]

    # Calculate the total path length
    total_path_length = 0.0
    for i in range(1, len(positions)):
        # Euclidean distance between consecutive positions
        p1 = np.array(positions[i-1])
        p2 = np.array(positions[i])
        distance = np.linalg.norm(p2 - p1)
        total_path_length += distance

    bag.close()
    return total_path_length

import csv

def get_seq_stats(directory_root_path, directories, odom_topic, output_csv_path):
    with open(output_csv_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Sequence', 'File', 'Duration (min:sec)', 'Path Length (meters)'])
        
        total_duration = 0
        total_distance_covered = 0
        for directory in directories:
            directory_path = os.path.join(directory_root_path, directory, "liosam")
            if os.path.exists(directory_path) and os.path.isdir(directory_path):
                files = os.listdir(directory_path)
                files.sort(key=lambda x: int(re.search(r'(\d+)\_liosam.bag$', x).group(1)) if re.search(r'(\d+)\_liosam.bag$', x) else 0)

                for file in files:
                    if file.endswith(".bag"):
                        bag_path = os.path.join(directory_path, file)
                        duration = get_bag_duration(bag_path)
                        minutes, seconds = divmod(int(duration), 60)

                        path_length = get_bag_path_length(bag_path, odom_topic)

                        # Log to CSV
                        writer.writerow([directory, file, f"{minutes}:{seconds:02d}", f"{path_length:.2f}"])
                        print(f"Writing to CSV: {directory}, {file}, Duration: {minutes}:{seconds:02d}, Path Length: {path_length:.2f}")

                        total_duration += duration
                        total_distance_covered += path_length

        total_minutes, total_seconds = divmod(int(total_duration), 60)
        writer.writerow(['Total', '', f"{total_minutes}:{total_seconds:02d}", f"{total_distance_covered:.2f}"])