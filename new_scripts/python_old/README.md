ColoRadar Python tools
======================

# 1. Description

This package includes a set of Python tools designed to load and process the ColoRadar radar state estimation dataset. In addition, this package includes an example script that demonstrates how to use the dataset tools to animate plots of the sensor measurements from the dataset.

# 2. Dependencies

 - Numpy
 - SciPy

# 3. Dataset Tools

## 3.1 Data Loaders

Descriptions of the functions in ```dataset_loaders.py``` are given below.

- dataset_loaders.**get_adc_frame(index, seq_dir, params)**
  - **Parameters:**
    - **index** (*int*): the index of the adc data frame to load
    - **seq_dir** (*str*): the base directory for the sequence
    - **params** (*dict*): waveform parameters obtained from the ```get_<radar_sensor>_params()``` function
  - **Returns:**
    - **frame** (*Numpy array*): complex ADC samples from the requested frame with dimension ```num_tx * num_rx * num_chirps_per_frame * num_adc_samples_per_chirp```
- dataset_loaders.**get_heatmap(index, seq_dir, params)**
  - **Parameters:**
    - **index** (*int*): the index of the heatmap to load
    - **seq_dir** (*str*): the base directory for the sequence
    - **params** (*dict*): heatmap parameters obtained from the ```get_<radar_sensor>_params()``` function
  - **Returns:**
    - **frame** (*Numpy array*): heatmap for the requested frame with dimension ```num_elevation_bins * num_azimuth_bins * num_range_bins * 2```. Each elevation, azimuth, and range index includes 2 values: the real-valued intensity of the doppler peak at that location and the measured range-rate at that location in meters per second
- dataset_loaders.**get_pointcloud(index, seq_dir, params)**
  - **Parameters:**
    - **index** (*int*): the index of the frame to load
    - **seq_dir** (*str*): the base directory for the sequence
    - **params** (*dict*): pointcloud parameters obtained from the ```get_<radar_or_lidar_sensor>_params()``` function
  - **Returns:**
    - **cloud** (*Numpy array*): pointcloud matrix with one point per row. Dimension is ```num_points * 5``` for radar (x,y,z,intensity,doppler) and ```num_points * 4``` for lidar (x,y,z,intensity)
- dataset_loaders.**get_timestamps(seq_dir, params)**
  - **Parameters:**
    - **seq_dir** (*str*): the base directory for the sequence
    - **params** (*dict*): sensor parameters obtained from the ```get_<sensor>_params()``` function
  - **Returns:**
    - **stamps** (*list[float]*): timestamps for each sensor measurement in seconds
- dataset_loaders.**get_imu(seq_dir)**
  - **Parameters:**
    - **seq_dir** (*str*): the base directory for the sequence
  - **Returns:**
    - **imu_data** (*list[dict]*): list of imu measurements where each measurement is stored as a dict in which ```measurement['accel']``` stores the linear acceleration in meters per second and ```measurement['gyro']``` stores the angular rates in radians per second
- dataset_loaders.**get_groundtruth(seq_dir)**
  - **Parameters:**
    - **seq_dir** (*str*): the base directory for the sequence
  - **Returns:**
    - **gt_data** (*list[dict]*): list of groundtruth measurements where each measurement is stored as a dict in which ```measurement['position']``` stores the position (x,y,z) in meters and ```measurement['orientation']``` stores the orientation as a quaternion (x,y,z,w)
- dataset_loaders.**get_vicon(seq_dir)**
  - **Parameters:**
    - **seq_dir** (*str*): the base directory for the sequence
  - **Returns:**
    - **gt_data** (*list[dict]*): list of vicon measurements where each measurement is stored as a dict in which ```measurement['position']``` stores the position (x,y,z) in meters and ```measurement['orientation']``` stores the orientation as a quaternion (x,y,z,w)
- dataset_loaders.**get_lidar_params(calib_dir)**
  - **Parameters:**
    - **calib_dir** (*str*): the base directory for sensor calibrations
  - **Returns:**
    - **params** (*dict*): lidar sensor parameters 
- dataset_loaders.**get_imu_params(calib_dir)**
  - **Parameters:**
    - **calib_dir** (*str*): the base directory for sensor calibrations
  - **Returns:**
    - **params** (*dict*): imu sensor parameters
- dataset_loaders.**get_groundtruth_params()**
  - **Returns:**
    - **params** (*dict*): groundtruth parameters
- dataset_loaders.**get_vicon_params(calib_dir)**
  - **Parameters:**
    - **calib_dir** (*str*): the base directory for sensor calibrations
  - **Returns:**
    - **params** (*dict*): vicon parameters
- dataset_loaders.**get_single_chip_params(calib_dir)**
  - **Parameters:**
    - **calib_dir** (*str*): the base directory for sensor calibrations
  - **Returns:**
    - **params** (*dict{dict}*): contains five sub-dictionaries
      - ```params['waveform']```: waveform parameters for the single-chip sensor
      - ```params['heatmap']```: heatmap parameters for the single-chip sensor
      - ```params['pointcloud']```: pointcloud parameters for the single-chip sensor
      - ```params['antenna_cfg']```: antenna layout for the single-chip sensor, required for angle-of-arrival processing of the adc data
      - ```params['coupling']```: antenna coupling calibration for the single chip sensor, required to remove antenna coupling effects when processing the adc data
- dataset_loaders.**get_cascade_params(calib_dir)**
  - **Parameters:**
    - **calib_dir** (*str*): the base directory for sensor calibrations
  - **Returns:**
    - **params** (*dict{dict}*): contains five sub-dictionaries
      - ```params['waveform']```: waveform parameters for the cascaded sensor
      - ```params['heatmap']```: heatmap parameters for the cascaded sensor
      - ```params['antenna_cfg']```: antenna layout for the cascaded sensor, required for angle-of-arrival processing of the adc data
      - ```params['coupling']```: antenna coupling calibration for the cascaded sensor, required to remove antenna coupling effects when processing the adc data
      - ```params['phase']```: phase calibration data for the cascaded sensor, required for accurate angle-of-arrival processing
      - ```params['frequency']```: frequency calibration data for the cascaded sensor, required for accurate angle-of-arrival processing

## 3.2 Calibration Examples

Examples describing how to apply antenna coupling, phase, and frequency calibrations are given in ```calibration.py```:

- calibration.**blackman(i,n)**
  - **Parameters:**
    - **i** (*float*): the sample index 
    - **n** (*float*): the window size
  - **Returns:**
    - **sample** (*float*): the value of sample index ```i``` in a Blackman window of size ```n```
- calibration.**apply_coupling_calibration(adc_samples, coupling_calib)**
  - **Parameters:**
    - **adc_samples** (*Numpy array*): complex ADC samples for a frame of radar data obtained from ```dataset_loaders.get_adc_frame()```
    - **coupling_calib** (*dict*): antenna coupling calibration obtained from ```dataset_loaders.get_<radar_sensor>_params()```
  - **Returns:**
    - **calibrated_range_fft_data** (*Numpy array*): post-range-fft radar data with antenna coupling calibration applied
- calibration.**apply_phase_calibration(adc_samples, phase_calib)***
  - **Parameters:**
    - **adc_samples** (*Numpy array*): complex ADC samples for a frame of radar data obtained from ```dataset_loaders.get_adc_frame()```
    - **phase_calib** (*dict*): phase calibration obtained from ```dataset_loaders.get_cascade_params()```
  - **Returns:**
    - **phase_calibrated_adc_samples** (*Numpy array*): ADC samples with phase calibration applied
- calibration.**apply_frequency_calibration(adc_samples, phase_calib, wave_config)***
  - **Parameters:**
    - **adc_samples** (*Numpy array*): complex ADC samples for a frame of radar data obtained from ```dataset_loaders.get_adc_frame()```
    - **freq_calib** (*dict*): frequency calibration obtained from ```dataset_loaders.get_cascade_params()```
    - **wave_config** (*dict*): waveform config obtained from ```dataset_loaders.get_cascade_params()```
  - **Returns:**
    - **freq_calibrated_adc_samples** (*Numpy array*): ADC samples with frequency calibration applied

## 3.3 Example Animation Script

The script ```plot_pointclouds.py``` is provided to demonstrate the use of the functions provided in ```dataset_loader.py```. It animates plots of the sensor rig's position, lidar measurements, and radar measurements for the specified sensor and data type.

- **Arguments:**
  - **-s (--seq)** *(str)*: the base directory for the sequence to be animated
  - **-c (--calib)** *(str)*: the base directory for the sensor calibrations
  - **-t (--threshold)** *(float)*: normalized intensity threshold for plotting heatmap points, should be between [0.0,1.0], defaults to 0.2
  - **-mr (--min_range)** *(int)*: minimum range bin to plot if plotting heatmaps, defaults to 10
  - **-hm (--plot_heatmap)** *(bool)*: true to plot radar heatmaps, false to plot pointclouds, defaults to false
  - **-sc (--single_chip)** *(bool)*: true to plot single chip data, false to plot cascaded data

An example image from the animation is shown below. Lidar points are shown in red, radar heatmap points are colormapped by intensity from dark purple to yellow, and the sensor rig positions are in green.

<img src="https://github.com/arpg/ColoRadar_tools/blob/master/img/lidar_radar1.png">