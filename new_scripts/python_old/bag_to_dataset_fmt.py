import rosbag
import argparse
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcl
import os
from array import array
import shutil

if __name__ == '__main__':
  
  parser = argparse.ArgumentParser(description='turn list of rosbags into ColoRadar dasatet format')
  parser.add_argument('-i', '--in_dir', type=str, help='directory in which input bag files are stored')
  parser.add_argument('-o', '--out_dir', type=str, help='base directory for output dataset')
  parser.add_argument('--cascade_raw', type=str, default='/cascade/data_cube')
  parser.add_argument('--cascade_heatmap', type=str, default='/cascade/heatmap')
  parser.add_argument('--ar_raw', type=str, default='/dca_node/data_cube')
  parser.add_argument('--ar_heatmap', type=str, default='/dca_node/heatmap')
  parser.add_argument('--ar_pointcloud', type=str, default='/mmWaveDataHdl/RScan')
  parser.add_argument('--lidar', type=str, default='/os1_cloud_node/points')
  parser.add_argument('--imu', type=str, default='/gx5/imu/data')
  parser.add_argument('--groundtruth', type=str, default='/carto_odom')
  parser.add_argument('--vicon', type=str, default='')
  args = parser.parse_args()

  if args.in_dir[-1] != '/':
    args.in_dir = args.in_dir + '/'
  if not os.path.isdir(args.in_dir):
    print('Input rosbag directory (' + args.in_dir + ') does not exist.')
    exit()

  if args.out_dir[-1] != '/':
    args.out_dir = args.out_dir + '/'
  if not os.path.isdir(args.out_dir):
    print('Output dataset directory (' + args.out_dir + ') does not exist.')
    exit()

  # get list of rosbags to convert
  bag_list = []
  for root, dirs, files in os.walk(args.in_dir):
    date = root.split('/')[-1]
    for file in files:
      if file.endswith('.bag'):
        bag_list.append((file,date))

  for bag_filename, date in bag_list:
    bag_name = bag_filename.split('.')[0]
    base_dir = args.out_dir + date + '_' + bag_name + '/'

    # check if directory structure already exists, create it if not
    if not os.path.isdir(base_dir):
      os.mkdir(base_dir)
    if not os.path.isdir(base_dir + 'cascade/'):
      os.mkdir(base_dir + 'cascade/')
    if not os.path.isdir(base_dir + 'cascade/adc_samples/'):
      os.mkdir(base_dir + 'cascade/adc_samples/')
    if not os.path.isdir(base_dir + 'cascade/adc_samples/data/'):
      os.mkdir(base_dir + 'cascade/adc_samples/data/')
    if not os.path.isdir(base_dir + 'cascade/heatmaps/'):
      os.mkdir(base_dir + 'cascade/heatmaps/')
    if not os.path.isdir(base_dir + 'cascade/heatmaps/data/'):
      os.mkdir(base_dir + 'cascade/heatmaps/data/')
    if not os.path.isdir(base_dir + 'single_chip/'):
      os.mkdir(base_dir + 'single_chip/')
    if not os.path.isdir(base_dir + 'single_chip/adc_samples/'):
      os.mkdir(base_dir + 'single_chip/adc_samples/')
    if not os.path.isdir(base_dir + 'single_chip/adc_samples/data/'):
      os.mkdir(base_dir + 'single_chip/adc_samples/data/')
    if not os.path.isdir(base_dir + 'single_chip/heatmaps/'):
      os.mkdir(base_dir + 'single_chip/heatmaps/')
    if not os.path.isdir(base_dir + 'single_chip/heatmaps/data/'):
      os.mkdir(base_dir + 'single_chip/heatmaps/data/')
    if not os.path.isdir(base_dir + 'single_chip/pointclouds/'):
      os.mkdir(base_dir + 'single_chip/pointclouds/')
    if not os.path.isdir(base_dir + 'single_chip/pointclouds/data'):
      os.mkdir(base_dir + 'single_chip/pointclouds/data')
    if not os.path.isdir(base_dir + 'lidar/'):
      os.mkdir(base_dir + 'lidar/')
    if not os.path.isdir(base_dir + 'lidar/pointclouds/'):
      os.mkdir(base_dir + 'lidar/pointclouds/')
    if not os.path.isdir(base_dir + 'imu/'):
      os.mkdir(base_dir + 'imu/')
    if not os.path.isdir(base_dir + 'groundtruth/'):
      os.mkdir(base_dir + 'groundtruth/')

    bag = rosbag.Bag(args.in_dir + date + '/' + bag_filename)

    # read imu data from bag and write to output files
    gen = bag.read_messages(topics=[args.imu])
    #gen = sorted(gen, key=lambda a: a[1].header.stamp.to_sec())
    imu_t_file = open(base_dir + 'imu/timestamps.txt', 'w')
    imu_data_file = open(base_dir + 'imu/imu_data.txt', 'w')
    for topic, msg, t in gen:
      imu_t_file.write('%6f\n' % (msg.header.stamp.to_sec()))
      a_x = msg.linear_acceleration.x
      a_y = msg.linear_acceleration.y
      a_z = msg.linear_acceleration.z
      alpha_x = msg.angular_velocity.x
      alpha_y = msg.angular_velocity.y
      alpha_z = msg.angular_velocity.z
      imu_data_file.write(str(a_x) + ' ' + str(a_y) + ' ' + str(a_z) + ' ' + 
                          str(alpha_x) + ' ' + str(alpha_y) + ' ' + str(alpha_z) + '\n')
    imu_t_file.close()
    imu_data_file.close()

    # read groundtruth data from bag and write to output files
    gen = bag.read_messages(topics=[args.groundtruth])
    #gen = sorted(gen, key=lambda a: a[1].header.stamp.to_sec())
    gt_t_file = open(base_dir + 'groundtruth/timestamps.txt', 'w')
    gt_data_file = open(base_dir + 'groundtruth/groundtruth_poses.txt', 'w')
    for topic, msg, t in gen:
      gt_t_file.write('%6f\n' % (msg.header.stamp.to_sec()))
      p_x = msg.pose.pose.position.x
      p_y = msg.pose.pose.position.y
      p_z = msg.pose.pose.position.z
      q_x = msg.pose.pose.orientation.x
      q_y = msg.pose.pose.orientation.y
      q_z = msg.pose.pose.orientation.z
      q_w = msg.pose.pose.orientation.w
      gt_data_file.write(str(p_x) + ' ' + str(p_y) + ' ' + str(p_z) + ' ' + 
                         str(q_x) + ' ' + str(q_y) + ' ' + str(q_z) + ' ' + str(q_w) + '\n')
    gt_t_file.close()
    gt_data_file.close()

    # assuming for now vicon publishes poseStamped messages,
    # shouldn't be too hard to change if needed
    if (args.vicon != ''):
      gen = bag.read_messages(topics=[args.vicon])
      vicon_t_file = open(base_dir + 'vicon/timestamps.txt', 'w')
      vicon_data_file = open(base_dir + 'vicon/vicon_poses.txt', 'w')
      for topic, msg, t in gen:
        vicon_t_file.write('%6f\n' * (msg.header.stamp.to_sec()))
        p_x = msg.pose.position.x
        p_y = msg.pose.position.y
        p_z = msg.pose.position.z
        q_x = msg.pose.orientation.x
        q_y = msg.pose.orientation.y
        q_z = msg.pose.orientation.z
        q_w = msg.pose.orientation.w
        vicon_data_file.write(str(p_x) + ' ' + str(p_y) + ' ' + str(p_z) + ' ' + 
                           str(q_x) + ' ' + str(q_y) + ' ' + str(q_z) + ' ' + str(q_w) + '\n')
      vicon_t_file.close()
      vicon_data_file.close()


    # read lidar data from bag and write to output files
    gen = bag.read_messages(topics=[args.lidar])
    #gen = sorted(gen, key=lambda a: a[1].header.stamp.to_sec())
    lidar_t_file = open(base_dir + 'lidar/timestamps.txt', 'w')
    msg_idx = 0
    for topic, msg, t in gen:
      lidar_t_file.write('%6f\n' % (msg.header.stamp.to_sec()))

      point_gen = pcl.read_points(msg,
                                  skip_nans=True,
                                  field_names=('x','y','z','intensity'))
      cloud_file = open(base_dir + 'lidar/pointclouds/lidar_pointcloud_' + str(msg_idx) + '.bin', 'wb')
      for point in point_gen:
        arr = array('f', point)
        arr.tofile(cloud_file)
      cloud_file.close()

      msg_idx += 1

    lidar_t_file.close()

    # read single chip radar pointclouds from bag and write to output files
    gen = bag.read_messages(topics=[args.ar_raw])
    #gen = sorted(gen, key=lambda a: a[1].header.stamp.to_sec())
    single_chip_raw_t_file = open(base_dir + 'single_chip/adc_samples/timestamps.txt', 'w')
    msg_idx = 0
    for topic, msg, t in gen:
      
      single_chip_raw_t_file.write('%6f\n' % (msg.header.stamp.to_sec()))

      raw_file = open(base_dir + 'single_chip/adc_samples/data/frame_' + str(msg_idx) + '.bin', 'wb')
      arr = array('h', msg.samples)
      arr.tofile(raw_file)
      raw_file.close()

      msg_idx += 1

    single_chip_raw_t_file.close()

    # read single chip radar pointclouds from bag and write to output files
    gen = bag.read_messages(topics=[args.ar_pointcloud])
    #gen = sorted(gen, key=lambda a: a[1].header.stamp.to_sec())
    single_chip_pc_t_file = open(base_dir + 'single_chip/pointclouds/timestamps.txt', 'w')
    msg_idx = 0
    for topic, msg, t in gen:

      single_chip_pc_t_file.write('%6f\n' % (msg.header.stamp.to_sec()))

      point_gen = pcl.read_points(msg,
                                  skip_nans=True,
                                  field_names=('x','y','z','intensity','doppler'))
      cloud_file = open(base_dir + 'single_chip/pointclouds/data/radar_pointcloud_' + str(msg_idx) + '.bin', 'wb')
      for point in point_gen:
        arr = array('f', point)
        arr.tofile(cloud_file)
      cloud_file.close()

      msg_idx += 1

    single_chip_pc_t_file.close()
    
    # read single chip radar pointclouds from bag and write to output files
    gen = bag.read_messages(topics=[args.ar_heatmap])
    #gen = sorted(gen, key=lambda a: a[1].header.stamp.to_sec())
    single_chip_hm_t_file = open(base_dir + 'single_chip/heatmaps/timestamps.txt', 'w')
    msg_idx = 0
    for topic, msg, t in gen:

      single_chip_hm_t_file.write('%6f\n' % (msg.header.stamp.to_sec()))

      heatmap_file = open(base_dir + 'single_chip/heatmaps/data/heatmap_' + str(msg_idx) + '.bin', 'wb')
      arr = array('f', msg.image)
      arr.tofile(heatmap_file)
      heatmap_file.close()

      msg_idx += 1

    single_chip_hm_t_file.close()
    

    # read cascade radar data from bag and write to output files
    gen = bag.read_messages(topics=[args.cascade_raw])
    #gen = sorted(gen, key=lambda a: a[1].header.stamp.to_sec())
    cascade_raw_t_file = open(base_dir + 'cascade/adc_samples/timestamps.txt', 'w')
    msg_idx = 0
    for topic, msg, t in gen:

      cascade_raw_t_file.write('%6f\n' % (msg.header.stamp.to_sec()))

      raw_file = open(base_dir + 'cascade/adc_samples/data/frame_' + str(msg_idx) + '.bin', 'wb')
      arr = array('h', msg.samples)
      arr.tofile(raw_file)
      raw_file.close()

      msg_idx += 1

    cascade_raw_t_file.close()
    
    # read cascade radar data from bag and write to output files
    gen = bag.read_messages(topics=[args.cascade_heatmap])
    #gen = sorted(gen, key=lambda a: a[1].header.stamp.to_sec())
    cascade_hm_t_file = open(base_dir + 'cascade/heatmaps/timestamps.txt', 'w')
    msg_idx = 0
    for topic, msg, t in gen:

      cascade_hm_t_file.write('%6f\n' % (msg.header.stamp.to_sec()))

      heatmap_file = open(base_dir + 'cascade/heatmaps/data/heatmap_' + str(msg_idx) + '.bin', 'wb')
      arr = array('f', msg.image)
      arr.tofile(heatmap_file)
      heatmap_file.close()

      msg_idx += 1

    cascade_hm_t_file.close()

    # create zipped archive for the directory
    # zipped archive does not include base directory
    '''
    shutil.make_archive(args.out_dir + date + '_' + bag_name, 
                        'zip', 
                        root_dir=base_dir,
                        base_dir='../')
    '''