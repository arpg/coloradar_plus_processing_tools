#!/usr/bin/env python3

import numpy as np
import struct
import os
import json 
import cv2 


def bin_to_jpg(bin_file, output_jpg, width, height, channels):
    # Step 1: Read the binary file
    with open(bin_file, 'rb') as f:
        # Load the binary data
        img_data = f.read()

    # Step 2: Convert the binary data to a NumPy array
    if channels == 1:
        img_array = np.frombuffer(img_data, dtype=np.uint16)  # 16-bit depth image
        img_array = img_array.reshape((height, width))
    else:
        img_array = np.frombuffer(img_data, dtype=np.uint8)
        img_array = img_array.reshape((height, width, channels))  # RGB or other formats

    if channels == 3:
        img_array = cv2.cvtColor(img_array, cv2.COLOR_BGR2RGB)

    # Step 4: Normalize the depth image for saving as JPG (optional)
    if channels == 1:
        # Normalize to the range [0, 255] for visualization (optional step)
        img_array = cv2.normalize(img_array, None, 0, 255, cv2.NORM_MINMAX)
        img_array = img_array.astype(np.uint8)  # Convert to 8-bit

    # Step 5: Save the image using OpenCV
    print(f"writing {output_jpg} ...")
    cv2.imwrite(output_jpg, img_array)


def load_bin_file(file_path, shape, dtype=np.float32):
    """
    Load binary file and reshape it into the desired shape.
    """
    return np.fromfile(file_path, dtype=dtype).reshape(shape)


def parse_img_filename(filename):
  underscore_idx = [i for i,x in enumerate(filename) if x =="_"][-1]
  file_no = filename[underscore_idx+1:-4] 
  out_filename = file_no.zfill(5) + ".jpg"
  return out_filename 


def load_rgb_images(seq_dir, output_dir): 
  if not os.path.exists(output_dir):
    os.mkdir(output_dir)

  rgb_dir = os.path.join(seq_dir,"camera/images/rgb") 
  for img_bin in os.listdir(rgb_dir): 
    filename = parse_img_filename(img_bin)
    img_bin_path = os.path.join(rgb_dir,img_bin) 
    out_path = os.path.join(output_dir,filename) 
    bin_to_jpg(img_bin_path,out_path,1280,800,3) #images are 1280x800 


def load_depth_images(seq_dir,output_dir): 
  if not os.path.exists(output_dir):
    os.mkdir(output_dir) 
  
  depth_dir = os.path.join(seq_dir,"camera/images/depth")
  for img_bin in os.listdir(depth_dir): 
    filename = parse_img_filename(img_bin)
    img_bin_path = os.path.join(depth_dir,img_bin) 
    out_path = os.path.join(output_dir,filename) 
    bin_to_jpg(img_bin_path,out_path,848,480,1)  


# reads raw adc data from bin file 
# param[in] index: index of the requested frame
# param[in] base_dir: base directory of the sequence
# param[in] params: dict object containing radar config parameters
# return    numpy array containing complex adc samples with dimensions 
#            num_tx * num_rx * chirps_per_frame * adc_samples_per_chirp
def load_adc_frame(index, seq_dir, params):
  if params['sensor_type'] == 'cascade':
    filename = seq_dir + '/cascade/adc_samples/data/frame_' + str(index) + '.bin'
  else:
    filename = seq_dir + '/single_chip/adc_samples/data/frame_' + str(index) + '.bin'

  if not os.path.exists(filename):
    print('File ' + filename + ' not found')
    return

  with open(filename, mode='rb') as file:
    frame_bytes = file.read()

  frame_vals = struct.unpack(str(len(frame_bytes) // 2)+'h', frame_bytes)
  frame_vals = np.array(frame_vals, dtype=float)
  frame_vals = frame_vals[:-1:2] + 1j * frame_vals[1::2]

  frame = frame_vals.reshape(params['num_tx'],
                             params['num_rx'],
                             params['num_chirps_per_frame'],
                             params['num_adc_samples_per_chirp'])
  return frame

# reads post-angle-fft heatmap from bin file
# param[in] index: index of the requested heatmap
# param[in] seq_dir: base directory of the sequence
# param[in] params: dict object containing radar config parameters
# return    numpy array containing real-valued intensities for the post-angle-fft heatmap
#           with dimensions num_elevation_bins * num_azimuth_bins * num_range_bins * 2
#           where each elevation-azimuth-range bin has 2 values: the peak intensity and
#           the peak location for the doppler spectrum for that bin
def load_heatmap(index, seq_dir, params):
  filename = seq_dir + '/cascade/heatmaps/data/heatmap_' + str(index) + '.bin'

  if not os.path.exists(filename):
    print('File ' + filename + ' not found')
    return None
  else:
    with open(filename, mode='rb') as file: 
      frame_bytes = file.read()
    frame_vals = struct.unpack(str(len(frame_bytes) // 4)+'f', frame_bytes)
    frame_vals = np.array(frame_vals)
    print(f"heatmap shape: {frame_vals.shape}")
    frame = frame_vals.reshape((params['num_elevation_bins'], params['num_azimuth_bins'], params['num_range_bins'], 2)) # 2 vals for each bin (doppler peak intensity and peak location)
    # frame = np.random.rand(10,4)
    return frame
    
# reads pointcloud from bin file
# param[in] index: index of the requested pointcloud
# param[in] seq_dir: base directory of the sequence
# param[in] params: dict object containing sensor config parameters
# return    a numpy array containing one point per row, 
#           lidar point values are [x,y,z,intensity]
#           radar point values are [x,y,z,intensity,doppler]
def load_pointcloud(index, seq_dir, params):
  print(f"index: {index}")
  if params['sensor_type'] == 'lidar':
    filename = seq_dir + '/lidar/pointclouds/lidar_pointcloud_' + str(index) + '.bin'

  if not os.path.exists(filename):
    print('File ' + filename + ' not found')
    return None

  # print(f"filename: {filename}")
  # cloud = np.fromfile(filename, dtype=np.float32).reshape((-1,5))

  with open(filename, mode='rb') as file:
    cloud_bytes = file.read()

  cloud_vals = struct.unpack(str(len(cloud_bytes) // 4)+'f', cloud_bytes)
  cloud_vals = np.array(cloud_vals)

  if params['sensor_type'] == 'lidar':
    cloud = cloud_vals.reshape((-1, 4))
    cloud = cloud[:, :4]
  return cloud


# reads all timestamps from file
# param[in] seq_dir: base directory of the sequence
# param[in] params: dict object containing sensor config parameters
# return    list of timestamps in seconds as floating point values
def load_timestamps(seq_dir, params):
  if params['sensor_type'] == 'lidar': 
    filename = seq_dir + '/lidar/timestamps.txt'
  elif params['sensor_type'] == 'imu':
    filename = seq_dir + '/imu/timestamps.txt'
  elif params['sensor_type'] == 'groundtruth':
    filename = seq_dir + '/groundtruth/timestamps.txt'
  else:
    if params['sensor_type'] == 'cascade':
      filename = seq_dir + '/cascade'
    else:
      filename = seq_dir + '/single_chip'
    if params['data_type'] == 'adc_samples':
      filename += '/adc_samples/timestamps.txt'
    elif params['data_type'] == 'heatmap':
      filename += '/heatmaps/timestamps.txt'
    else:
      filename += '/pointclouds/timestamps.txt'

  if not os.path.exists(filename):
    print('File ' + filename + ' not found')
    return

  with open(filename, mode='r') as file:
    lines = file.readlines()

  stamps = [float(line) for line in lines]

  return stamps


# reads all imu measurements from file
# param[in] seq_dir: base directory of the sequence
# return    list of imu measurements where each imu measurement is a dict with two entries:
#            {'accel': [x,y,z], 'gyro': [x,y,z]}
def load_imu(seq_dir):
  filename = seq_dir + '/imu/imu_data.txt'
  if not os.path.exists(filename):
    print('File ' + filename + ' not found')
    return

  with open(filename, mode='r') as file:
    lines = file.readlines()

  imu_data = []
  for line in lines:
    vals = [float(s) for s in line.split()]
    imu_data.append({'accel': vals[:3], 'gyro': vals[3:]})

  return imu_data

from scipy.spatial.transform import Rotation
def get_sensor_poses_from_gt(gt_data, sensor_params):
  tranformed_sensor_poses = []
  for gt_pose in gt_data:
    # print(f"gt_pose: {gt_pose}")
    tranformed_sensor_pose = {}

    gt_pose_4x4 = np.eye(4)
    gt_pose_4x4[:3,3] = gt_pose['position']
    gt_pose_4x4[:3,:3] = Rotation.from_quat(gt_pose['orientation']).as_matrix()

    sensor_to_base_4x4 = np.eye(4)
    sensor_to_base_4x4[:3,3] = sensor_params['translation']
    sensor_to_base_4x4[:3,:3] = Rotation.from_quat(sensor_params['rotation']).as_matrix()

    sensor_pose_global = gt_pose_4x4 @ sensor_to_base_4x4

    tranformed_sensor_pose['position'] = sensor_pose_global[:3,3]
    tranformed_sensor_pose['orientation'] = Rotation.from_matrix(sensor_pose_global[:3, :3]).as_quat()
    tranformed_sensor_poses.append(tranformed_sensor_pose)

  return tranformed_sensor_poses

# reads all groundtruth poses from file
# param[in] seq_dir: base directory of the sequence
# return    list of groundtruth poses where each pose is a dict with two entries:
#           {'position': [x,y,z], 'orientation': [x,y,z,w]}
def load_groundtruth(seq_dir):
  #filename = seq_dir + '/groundtruth/groundtruth_poses_quat.txt'
  filename = seq_dir + '/groundtruth/groundtruth_poses.txt'
  if not os.path.exists(filename):
    print('File ' + filename + ' not found')
    return

  with open(filename, mode='r') as file:
    lines = file.readlines()

  gt_data = []
  for line in lines:
    vals = [float(s) for s in line.split()]
    gt_data.append({'position': np.array(vals[:3]), 
                    'orientation': np.array(vals[3:])})

  return gt_data


# reads extrinsic transform data from a file
# param[in] filename: filename for the extrinsic transform file
# return t: the translation [x,y,z]
# return r: the rotation [x,y,z,w]
def load_tf_file(filename):
  if not os.path.exists(filename):
    print('File ' + filename + ' not found')
    return

  with open(filename, mode='r') as file:
    lines = file.readlines()

  t = [float(s) for s in lines[0].split()]
  r = [float(s) for s in lines[1].split()]

  return t, r


# reads lidar config and calibration data from a file
# param[in] calib_dir: base directory for dataset calibration files
# return params: dict with all lidar config and calibration parameters
def load_lidar_params(calib_dir):
  params = {'sensor_type': 'lidar', 'data_type': 'pointcloud'}
  filename = calib_dir + '/transforms/base_to_lidar.txt'
  params['translation'], params['rotation'] = load_tf_file(filename)
  # params['translation'] = np.zeros((3,)) 
  # params['rotation'] = np.zeros((4,)); params['rotation'][3] = 1 
  return params


# reads imu config and calibration data from a file
# param[in] calib_dir: base directory for dataset calibration files
# return params: dict with all imu config and calibration parameters
def load_imu_params(calib_dir):
  params = {'sensor_type': 'imu'}
  filename = calib_dir + '/transforms/base_to_imu.txt'
  params['translation'], params['rotation'] = load_tf_file(filename)
  return params


# gets config data for groundtruth, doesn't read it from a file 
# because there's only two parameters and they're always the same
# return params: dict with the only two needed parameters for groundtruth
def load_groundtruth_params():
  return {'sensor_type': 'groundtruth', 'data_type': 'pose'}


# # reads vicon config and calibration data from a file
# # param[in] calib_dir: base directory for dataset calibration files
# # return params: dict with all vicon config and calibration parameters
# def get_vicon_params(calib_dir):
#   params = {'sensor_type': 'vicon'}
#   filename = calib_dir + '/transforms/base_to_vicon.txt'
#   params['translation'], params['rotation'] = read_tf_file(filename)
#   return params


# reads the antenna layout from the given file
# param[in] filename: the filename of the antenna config file
# return antenna_cfg: a dict containing antenna layout parameters
def read_antenna_cfg(filename):
  antenna_cfg = {}

  with open(filename, mode='r') as file:
    lines = file.readlines()

  for line in lines:
    vals = line.split()

    if vals[0] != '#':
      if vals[0] == 'num_rx':
        antenna_cfg['num_rx'] = int(vals[1])
        antenna_cfg['rx_locations'] = [0] * antenna_cfg['num_rx']
      elif vals[0] == 'num_tx':
        antenna_cfg['num_tx'] = int(vals[1])
        antenna_cfg['tx_locations'] = [0] * antenna_cfg['num_tx']
      elif vals[0] == 'rx':
        antenna_cfg['rx_locations'][int(vals[1])] = (int(vals[2]), int(vals[3]))
      elif vals[0] == 'tx':
        antenna_cfg['tx_locations'][int(vals[1])] = (int(vals[2]), int(vals[3]))
      elif vals[0] =='F_design':
        antenna_cfg['F_design'] = float(vals[1])

  return antenna_cfg


# reads the heatmap config parameters from the given file and
# adds them to the given parameter dict
# param[in] hm_filename: the filename of the heatmap config file
# param[in] hm_params: dict containing the heatmap config
# return hm_params: heatmap config dict with heatmap config parameters added
def read_heatmap_cfg(hm_filename, hm_params):
  hm_params['data_type'] = 'heatmap'

  with open(hm_filename, mode='r') as file:
    lines = file.readlines()

  for line in lines:
    vals = line.split()
    # print("hm_params: ",hm_params.keys()) 
    # print("vals: ",vals) 
    if vals[0] == 'azimuth_bins' or vals[0] == 'elevation_bins':
      hm_params[vals[0]] = [float(s) for s in vals[1:]]
    elif vals[0] == 'range_bin_width':
      hm_params[vals[0]] = float(vals[1])
    else:
      hm_params[vals[0]] = int(vals[1])

  return hm_params


# reads the waveform config parameters from the given file and
# adds them to the given parameter dict
# param[in] wave_filename: the filename of the waveform config file
# param[in] wave_params: dict containing the waveform parameters
# return wave_params: waveform config dict with waveform config parameters added
def load_waveform_cfg(wave_filename, wave_params):
  wave_params['data_type'] = 'adc_samples'

  with open(wave_filename, mode='r') as file:
    lines = file.readlines()

  int_vals = ['num_rx', 'num_tx', 'num_adc_samples_per_chirp', 'num_chirps_per_frame']
  for line in lines:
    vals = line.split()
    if vals[0] in int_vals:
      wave_params[vals[0]] = int(vals[1])
    else:
      wave_params[vals[0]] = float(vals[1])

  return wave_params


# reads the antenna coupling calibration from the given file
# param[in] coupling_filename: the filename of the coupling config file
# return coupling_calib: dict containing antenna coupling calibration data
def load_coupling_cfg(coupling_filename):
  coupling_calib = {}

  with open(coupling_filename, mode='r') as file:
    lines = file.readlines()

  num_tx = int(lines[0].split(':')[1])
  num_rx = int(lines[1].split(':')[1])
  num_range_bins = int(lines[2].split(':')[1])
  coupling_calib['num_tx'] = num_tx
  coupling_calib['num_rx'] = num_rx
  coupling_calib['num_range_bins'] = num_range_bins
  coupling_calib['num_doppler_bins'] = int(lines[3].split(':')[1])
  data_str = lines[4].split(':')[1]
  data_arr = np.array(data_str.split(',')).astype('float')
  data_arr = data_arr[:-1:2] + 1j * data_arr[1::2]
  coupling_calib['data'] = data_arr.reshape(num_tx, num_rx, num_range_bins)

  return coupling_calib


# reads the phase and frequency calibration data from the given file
# param[in] calib_filename: the filename of the phase/frequency calibration file
# return phase_calib: dict containing the phase calibration data
# return freq_calib: dict containing the frequency calibration data
def load_phase_freq_calib(calib_filename):
  phase_calib = {}
  freq_calib = {}

  with open(calib_filename) as file: 
    data = json.load(file) 
  data = data['antennaCalib'] 

  # print("data: ",data)
  phase_cal_mat_arr = np.array(data['frequencyCalibrationMatrix']) 
  # print("phase_cal_mat_arr:",phase_cal_mat_arr.shape)
  phase_cal_mat = phase_cal_mat_arr.reshape(int(data['numTx']),int(data['numRx'])) 
  phase_cal_mat_arr = phase_cal_mat_arr[:-1:2] + 1j * phase_cal_mat_arr[1::2]
  # print("phase_cal_mat_arr: ",phase_cal_mat_arr)
  # print("phase_cal_mat_arr.shape: ",phase_cal_mat_arr.shape)
  

  phase_calib['num_rx'] = data['numRx']
  phase_calib['num_tx'] = data['numTx']
  phase_calib['cal_matrix']= phase_cal_mat 

  freq_calib['num_rx'] = data['numRx']
  freq_calib["num_tx"] = data['numTx']
  freq_calib['frequency_slope'] = data['frequencySlope']
  freq_calib['sampling_rate'] = data['samplingRate']
  freq_calib['cal_matrix'] = phase_cal_mat 

  '''
  with open(calib_filename) as file:
    lines = file.readlines()

  num_tx = int(lines[0].split(':')[1])
  num_rx = int(lines[1].split(':')[1])
  frequency_slope = float(lines[2].split(':')[1])
  sampling_rate = float(lines[3].split(':')[1])

  phase_cal_mat_str = lines[5].split(':')[1]
  phase_cal_mat_arr = np.array(phase_cal_mat_str.split(',')).astype('float')
  phase_cal_mat_arr = phase_cal_mat_arr[:-1:2] + 1j * phase_cal_mat_arr[1::2]
  phase_cal_mat = phase_cal_mat_arr.reshape(num_tx, num_rx)

  phase_calib['num_rx'] = num_rx
  phase_calib['num_tx'] = num_tx
  phase_calib['cal_matrix'] = phase_cal_mat

  freq_cal_mat_str = lines[4].split(':')[1]
  freq_cal_mat_arr = np.array(freq_cal_mat_str.split(',')).astype('float')
  freq_cal_mat = freq_cal_mat_arr.reshape(num_tx, num_rx)
plete.bag
        - Compressing and removing uncompressed directory: /media/kristen/easystore2/proc
  freq_calib['num_rx'] = num_rx
  freq_calib['num_tx'] = num_tx
  freq_calib['frequency_slope'] = frequency_slope
  freq_calib['sampling_rate'] = sampling_rate
  freq_calib['cal_matrix'] = freq_cal_mat
  '''

  return phase_calib, freq_calib


# gets config and calibration data for the cascaded radar sensor
# param[in] calib_dir: base directory for dataset calibration files
# return dict including waveform, heatmap, antenna, coupling, 
#             phase, and frequency calibration data
def load_cascade_params(calib_dir):
  wave_params = {'sensor_type': 'cascade'}
  hm_params = {'sensor_type': 'cascade'}

  tf_filename = calib_dir + '/transforms/base_to_radar.txt'

  t, r = load_tf_file(tf_filename)

  wave_params['translation'] = t
  wave_params['rotation'] = r
  hm_params['translation'] = t
  hm_params['rotation'] = r

  wave_filename = calib_dir + '/cascade/waveform_cfg.txt'
  wave_params = load_waveform_cfg(wave_filename, wave_params)

  hm_filename = calib_dir + '/cascade/heatmap_cfg.txt'
  hm_params = read_heatmap_cfg(hm_filename, hm_params)

  antenna_filename = calib_dir + '/cascade/antenna_cfg.txt'
  antenna_cfg = read_antenna_cfg(antenna_filename)

  #coupling_filename = calib_dir + '/cascade/coupling_calib.txt'
  #coupling_calib = read_coupling_cfg(coupling_filename)
  coupling_calib = {} # placeholder since I haven't done a coupling calibration for the cascade sensor yet

  phase_filename = calib_dir + '/cascade/phase_frequency_calib.txt'
  phase_calib, freq_calib = load_phase_freq_calib(phase_filename)

  return {'waveform': wave_params, 
          'heatmap': hm_params,  
          'antenna': antenna_cfg, 
          'coupling': coupling_calib,
          'phase': phase_calib,
          'frequency': freq_calib} 