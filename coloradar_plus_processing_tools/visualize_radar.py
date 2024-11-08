import h5py
import numpy as np
import open3d as o3d
import argparse
from scipy.spatial.transform import Rotation as R
import time


def load_data(h5_file_path, occupancy_threshold=0.6):
    """Load lidar map, poses, and radar clouds from the HDF5 file with filtering."""
    with h5py.File(h5_file_path, "r") as h5file:
        poses = h5file["poses"][:]
        lidar_map_data = h5file["lidar_map"][:]
        radar_clouds = []

        for i, pose in enumerate(poses):
            radar_key = f"radar_cloud_{i}"
            if radar_key in h5file:
                radar_data = h5file[radar_key][:]
                radar_clouds.append(np.array(radar_data))

    radar_clouds = np.array(radar_clouds)
    probabilities = 1 - (1 / (1 + np.exp(lidar_map_data[:, 3])))
    lidar_map_data = lidar_map_data[probabilities >= occupancy_threshold]
    lidar_map_points = lidar_map_data[:, :3]

    return lidar_map_points, poses, radar_clouds


def pose_to_matrix(pose):
    """Convert a pose (3D position + quaternion) into a 4x4 transformation matrix."""
    translation = pose[:3]
    quaternion = pose[3:]  # Quaternion as [x, y, z, w]
    rotation = R.from_quat(quaternion).as_matrix()  # 3x3 rotation matrix from quaternion

    # Create a 4x4 transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation
    transformation_matrix[:3, 3] = translation

    return transformation_matrix


def transform_points(points, transformation_matrix):
    """Apply a transformation matrix to a set of 3D points."""
    points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
    transformed_points = transformation_matrix @ points_homogeneous.T
    return transformed_points[:3, :].T


def set_camera_position(vis, pose, fixed_elevation=15.0):
    ctr = vis.get_view_control()
    transformation_matrix = pose_to_matrix(pose)
    target_position = transformation_matrix[:3, 3]
    camera_position = np.array([target_position[0], target_position[1], fixed_elevation])

    ctr.set_lookat(target_position)
    ctr.set_front([0, 0, -1])
    ctr.set_up([0, 1, 0])

    parameters = ctr.convert_to_pinhole_camera_parameters()
    new_extrinsic = np.array(parameters.extrinsic)
    new_extrinsic[:3, 3] = camera_position
    parameters.extrinsic = new_extrinsic
    ctr.convert_from_pinhole_camera_parameters(parameters)


def downsample_points(points, voxel_size=0.1):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return np.asarray(downsampled_pcd.points)


def create_sphere_at_point(point, radius=0.1, resolution=20, intensity=1.0):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=resolution)
    sphere.translate(point)
    sphere.compute_vertex_normals()
    color = np.array([intensity, intensity * 0.5, 0])  # Red channel proportional to intensity, dim green channel
    sphere.paint_uniform_color(color)
    return sphere


def show_frame(radar_points, pose, vis, radar_point_radius=0.05, intensity_threshold_percent=50):
    # print(f'got {len(radar_points)} points')
    frame_max_intensity = np.max(radar_points[:, 3])
    radar_points_filtered = radar_points[radar_points[:, 3] >= frame_max_intensity * intensity_threshold_percent / 100]
    radar_points_filtered[:, 3] /= frame_max_intensity
    points, intensities = radar_points_filtered[:, :3], radar_points_filtered[:, 3]
    # print(f'rendering {len(points)} points')

    transformation_matrix = pose_to_matrix(pose)
    transformed_points = transform_points(points, transformation_matrix)

    radar_spheres = []
    for point, intensity in zip(transformed_points, intensities):
        sphere = create_sphere_at_point(point, radius=radar_point_radius, intensity=intensity)
        radar_spheres.append(sphere)
        vis.add_geometry(sphere)

    return radar_spheres


def visualize(lidar_map_points, poses, radar_clouds, delay=0.2, lidar_voxel_size=0.05, frame_idx=None):
    lidar_map_points_downsampled = downsample_points(lidar_map_points, voxel_size=lidar_voxel_size)
    lidar_map = o3d.geometry.PointCloud()
    lidar_map.points = o3d.utility.Vector3dVector(lidar_map_points_downsampled)
    lidar_colors = np.array([[0.6, 0.6, 0.6, 0.5]] * len(lidar_map_points_downsampled))
    lidar_map.colors = o3d.utility.Vector3dVector(lidar_colors[:, :3])

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(lidar_map)
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])
    opt.point_size = 1

    # Trajectory setup
    trajectory_points = [poses[0][:3]]  # Initial point
    trajectory_lines = []
    trajectory = None
    # trajectory = o3d.geometry.LineSet()
    # trajectory.colors = o3d.utility.Vector3dVector([[0, 1, 0]])  # Green color for the trajectory


    if frame_idx is not None:
        show_frame(radar_clouds[frame_idx], poses[frame_idx], vis)
        vis.poll_events()
        vis.update_renderer()
        vis.run()
        return

    add = False

    for i, (pose, radar_cloud) in enumerate(zip(poses, radar_clouds)):
        radar_points = show_frame(radar_cloud, pose, vis)
        current_point = pose[:3]

        # if i > 0 and not np.allclose(current_point, trajectory_points[-1]):
        #     if trajectory is not None:
        #         vis.remove_geometry(trajectory)
        #     trajectory = o3d.geometry.LineSet()
        #     trajectory_points.append(current_point)
        #     trajectory_lines.append([len(trajectory_points) - 2, len(trajectory_points) - 1])
        #     trajectory.points = o3d.utility.Vector3dVector(trajectory_points)
        #     trajectory.lines = o3d.utility.Vector2iVector(trajectory_lines)
        #     trajectory.colors = o3d.utility.Vector3dVector([[0, 1, 0]])
        #     vis.add_geometry(trajectory)
        #
        # print('Iteration:', i, 'Current point:', current_point)
        # print('Trajectory points count:', len(trajectory_points))

        set_camera_position(vis, pose)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(delay)

        for point in radar_points:
            vis.remove_geometry(point)

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Display and animate the lidar map, radar clouds, and poses from an HDF5 file")
    parser.add_argument("file", type=str, help="Path to the HDF5 file containing lidar and radar data")
    parser.add_argument("--occupancy_threshold", type=float, default=0.75, help="Minimum probability for lidar points to be considered occupied")
    parser.add_argument("--intensity_threshold_percent", type=float, default=1.0, help="Percentage of max intensity for radar point inclusion")
    parser.add_argument("--delay", type=float, default=0.5, help="Delay between frames in seconds")
    parser.add_argument("-i", type=int, default=None, help="Static frame index")
    args = parser.parse_args()

    # Load lidar map, poses, and radar clouds with filtering
    lidar_map_points, poses, radar_clouds = load_data(args.file, args.occupancy_threshold)
    if args.i is not None and (not isinstance(args.i, int) or args.i >= len(poses)):
        raise ValueError(f"Invalid frame index: expected integer from 0 to {len(poses) - 1}, got {args.i}")

    # Visualize and animate the lidar map with camera following the poses
    visualize(lidar_map_points, poses, radar_clouds, args.delay, frame_idx=args.i)
