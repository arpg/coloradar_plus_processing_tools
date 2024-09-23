import rosbag
import argparse
import numpy as np
import os

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


if __name__ == '__main__':
  
  parser = argparse.ArgumentParser(description='turn list of rosbags into ColoRadar dasatet format')
  parser.add_argument('-b', '--bag', type=str, help='bag file from which to create calib files')
  parser.add_argument('-o', '--out_dir', type=str, help='base directory for output dataset')
  parser.add_argument('--cascade_raw', type=str, default='/cascade/data_cube')
  parser.add_argument('--cascade_heatmap', type=str, default='/cascade/heatmap')
  parser.add_argument('--ar_raw', type=str, default='/dca_node/data_cube')
  parser.add_argument('--ar_heatmap', type=str, default='/dca_node/heatmap')
  parser.add_argument('--ar_pointcloud', type=str, default='/mmWaveDataHdl/RScan')
  parser.add_argument('--lidar', type=str, default='/os1_cloud_node/points')
  parser.add_argument('--imu', type=str, default='/gx5/imu/data')
  args = parser.parse_args()

  if args.out_dir[-1] != '/':
    args.out_dir = args.out_dir + '/'
  if not os.path.isdir(args.out_dir):
    print('Output dataset directory (' + args.out_dir + ') does not exist.')
    exit()

  bag = rosbag.Bag(args.bag) 

  base_dir = args.out_dir + 'calib/'

  if not os.path.isdir(base_dir):
    os.mkdir(base_dir)
  if not os.path.isdir(base_dir + 'transforms/'):
    os.mkdir(base_dir + 'transforms/')
  if not os.path.isdir(base_dir + 'cascade/'):
    os.mkdir(base_dir + 'cascade/')
  if not os.path.isdir(base_dir + 'single_chip/'):
    os.mkdir(base_dir + 'single_chip/')

  gen = bag.read_messages(topics=[args.cascade_raw])
  item = next(gen, None)
  if item is not None:
    msg = item[1]
    cascade_wave_file = open(base_dir + 'cascade/waveform_cfg.txt', 'w')
    cascade_wave_file.write('num_rx ' + str(msg.num_rx) + '\n')
    cascade_wave_file.write('num_tx ' + str(msg.num_tx) + '\n')
    cascade_wave_file.write('num_adc_samples_per_chirp ' + str(msg.num_adc_samples_per_chirp) + '\n')
    cascade_wave_file.write('num_chirps_per_frame ' + str(msg.num_chirps_per_frame) + '\n')
    cascade_wave_file.write('adc_sample_frequency ' + str(msg.adc_sample_frequency) + '\n')
    cascade_wave_file.write('start_frequency ' + str(msg.start_frequency) + '\n')
    cascade_wave_file.write('idle_time ' + str(msg.idle_time) + '\n')
    cascade_wave_file.write('adc_start_time ' + str(msg.adc_start_time) + '\n')
    cascade_wave_file.write('ramp_end_time ' + str(msg.ramp_end_time) + '\n')
    cascade_wave_file.write('frequency_slope ' + str(msg.frequency_slope) + '\n')
    cascade_wave_file.close()

  gen = bag.read_messages(topics=[args.ar_raw])
  item = next(gen, None)
  if item is not None:
    msg = item[1]
    ar_wave_file = open(base_dir + 'single_chip/waveform_cfg.txt', 'w')
    ar_wave_file.write('num_rx ' + str(msg.num_rx) + '\n')
    ar_wave_file.write('num_tx ' + str(msg.num_tx) + '\n')
    ar_wave_file.write('num_adc_samples_per_chirp ' + str(msg.num_adc_samples_per_chirp) + '\n')
    ar_wave_file.write('num_chirps_per_frame ' + str(msg.num_chirps_per_frame) + '\n')
    ar_wave_file.write('adc_sample_frequency ' + str(msg.adc_sample_frequency) + '\n')
    ar_wave_file.write('start_frequency ' + str(msg.start_frequency) + '\n')
    ar_wave_file.write('idle_time ' + str(msg.idle_time) + '\n')
    ar_wave_file.write('adc_start_time ' + str(msg.adc_start_time) + '\n')
    ar_wave_file.write('ramp_end_time ' + str(msg.ramp_end_time) + '\n')
    ar_wave_file.write('frequency_slope ' + str(msg.frequency_slope) + '\n')
    ar_wave_file.close()

  gen = bag.read_messages(topics=[args.cascade_heatmap])
  item = next(gen, None)
  if item is not None:
    msg = item[1]
    cascade_hm_file = open(base_dir + 'cascade/heatmap_cfg.txt', 'w')
    cascade_hm_file.write('num_range_bins ' + str(msg.depth) + '\n')
    cascade_hm_file.write('num_elevation_bins ' + str(msg.height) + '\n')
    cascade_hm_file.write('num_azimuth_bins ' + str(msg.width) + '\n')
    #cascade_hm_file.write('num_doppler_bins ' + str(msg.num_doppler_bins) + '\n')
    cascade_hm_file.write('range_bin_width ' + str(msg.range_bin_width) + '\n')
    #cascade_hm_file.write('doppler_bin_width ' + str(msg.doppler_bin_width) + '\n')
    cascade_hm_file.write('azimuth_bins')
    for i in range(msg.width):
      cascade_hm_file.write(' ' + str(msg.azimuth_bins[i]))
    cascade_hm_file.write('\n')
    cascade_hm_file.write('elevation_bins')
    for i in range(msg.height):
      cascade_hm_file.write(' ' + str(msg.elevation_bins[i]))
    cascade_hm_file.write('\n')
    cascade_hm_file.close()

  gen = bag.read_messages(topics=[args.ar_heatmap])
  item = next(gen, None)
  if item is not None:
    msg = item[1]
    ar_hm_file = open(base_dir + 'single_chip/heatmap_cfg.txt', 'w')
    ar_hm_file.write('num_range_bins ' + str(msg.depth) + '\n')
    ar_hm_file.write('num_elevation_bins ' + str(msg.height) + '\n')
    ar_hm_file.write('num_azimuth_bins ' + str(msg.width) + '\n')
    #ar_hm_file.write('num_doppler_bins ' + str(msg.num_doppler_bins) + '\n')
    ar_hm_file.write('range_bin_width ' + str(msg.range_bin_width) + '\n')
    #ar_hm_file.write('doppler_bin_width ' + str(msg.doppler_bin_width) + '\n')
    ar_hm_file.write('azimuth_bins')
    for i in range(msg.width):
      ar_hm_file.write(' ' + str(msg.azimuth_bins[i]))
    ar_hm_file.write('\n')
    ar_hm_file.write('elevation_bins')
    for i in range(msg.height):
      ar_hm_file.write(' ' + str(msg.elevation_bins[i]))
    ar_hm_file.write('\n')
    ar_hm_file.close()

  gen = bag.read_messages(topics=['/tf'])
  imu_tf = False
  lidar_tf = False
  single_chip_tf = False
  cascade_tf = False
  vicon_tf = False

  for topic, msg, t in gen:

    for tf in msg.transforms:
      if tf.header.frame_id == 'imu_viz_link':
        if tf.child_frame_id == 'base_radar_link' and not single_chip_tf:
          single_chip_tf = True
          write_transform(tf, base_dir + 'transforms/base_to_single_chip.txt')
        elif tf.child_frame_id == 'gx5_link' and not imu_tf:
          imu_tf = True
          write_transform(tf, base_dir + 'transforms/base_to_imu.txt')
        elif tf.child_frame_id == 'cascade_link' and not cascade_tf:
          cascade_tf = True
          write_transform(tf, base_dir + 'transforms/base_to_cascade.txt')
        elif tf.child_frame_id == 'os1_sensor' and not lidar_tf:
          lidar_tf = True
          # fixed transform from os1_sensor to os1_lidar
          offset = (np.array([0,0,0.03618]),np.array([0,0,1,0]))
          write_transform(tf, base_dir + 'transforms/base_to_lidar.txt', offset)
        elif tf.child_frame_id == 'vicon_frame' and not vicon_tf:
          write_transform(tf, base_dir + 'transforms/base_to_vicon.txt')
        
    if imu_tf and lidar_tf and single_chip_tf and cascade_tf and vicon_tf:
      break