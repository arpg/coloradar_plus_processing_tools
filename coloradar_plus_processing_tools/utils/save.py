#!/usr/bin/env python3

import numpy as np


def save_pose_as_bin(pose_matrix, filename):
    """
    Save a 4x4 pose matrix as a binary file.
    """
    pose_matrix.astype(np.float32).tofile(filename)


def save_transform(trans, quat, filename):
    file = open(filename, 'w')
    file.write(str(trans[0]) + ' ' + str(trans[1]) + ' ' + str(trans[2]) + '\n')
    file.write(str(quat[0]) + ' ' + str(quat[1]) + ' ' + str(quat[2]) + ' ' + str(quat[3]) + '\n')
    file.close()


def save_cascade_datacube_config(msg, file_path):
    cascade_datacube_config_file = open(file_path, 'w')
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

def save_cascade_heatmap_config(msg, file_path):
    cascade_heatmap_config_file = open(file_path, 'w')
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