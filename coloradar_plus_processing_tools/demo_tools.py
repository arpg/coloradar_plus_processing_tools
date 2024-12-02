import h5py
import json
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation


def read_h5_dataset(file_path):
    data_dict = {}
    with h5py.File(file_path, 'r') as f:
        config = json.loads(f['config'][()])
        data_content = config.get('data_content', [])
        runs = config.get('runs', [])
        for content in data_content:
            data_dict[content] = {}
            for run in runs:
                dataset_name = f"{content}_{run}"
                if dataset_name in f:
                    data_dict[content][run] = f[dataset_name][:]
                else:
                    print(f"Dataset {dataset_name} not found in the file.")
    return data_dict


def inverse_transform(transform):
    if transform.shape != (7,):
        raise ValueError("Input must be a 7-element numpy array [x, y, z, qx, qy, qz, qw]")

    translation = transform[:3]
    quaternion = transform[3:]
    quaternion /= np.linalg.norm(quaternion)
    rotation = Rotation.from_quat(quaternion)

    inv_transform = np.zeros(7)
    inv_rotation = rotation.inv()
    inv_transform[:3] = -inv_rotation.apply(translation)
    inv_transform[3:] = inv_rotation.as_quat()
    return inv_transform


def radar_bins_to_fov(radar_config, azimuth_fov_idx, elevation_fov_idx, range_bin_idx):
    # max_range = radar_config.num_range_bins * radar_config.range_bin_width
    if not 0 <= azimuth_fov_idx <= radar_config.num_azimuth_bins // 2 - 1:
        raise ValueError(f'Select azimuth FOV from 0 to {radar_config.num_azimuth_bins // 2 - 1}')
    if not 0 <= elevation_fov_idx <= radar_config.num_elevation_bins // 2 - 1:
        raise ValueError(f'Select azimuth FOV from 0 to {radar_config.num_elevation_bins // 2 - 1}')
    # if not 0 < range_meters <= max_range:
    #     raise ValueError(f'Select max range from 0 to {max_range}')
    if not 0 <= range_bin_idx < radar_config.num_range_bins:
        raise ValueError(f'Select range bin from 0 to {radar_config.num_range_bins}')
    azimuth_fov_degrees = np.round(np.degrees(-radar_config.azimuth_bins[radar_config.num_azimuth_bins // 2 - 1 - azimuth_fov_idx]), 1)
    elevation_fov_degrees = np.round(np.degrees(-radar_config.elevation_bins[radar_config.num_elevation_bins // 2 - 1 - elevation_fov_idx]), 1)
    range_meters = np.round(radar_config.range_bin_width * (range_bin_idx + 1), 2)
    return {
        'total_horizontal_fov': azimuth_fov_degrees * 2,
        'total_vertical_fov': elevation_fov_degrees * 2,
        'range': range_meters
    }


def show_occupancy_pcl(cloud, prob_threshold=0):
    probabilities = 1 - 1 / (1 + np.exp(cloud[:, 3]))
    mask = probabilities >= prob_threshold
    filtered_points = cloud[:, :3][mask]
    filtered_probs = probabilities[mask]
    fp_max, fp_min = filtered_probs.max(), filtered_probs.min()
    normalized_probs = (filtered_probs - fp_min) / (fp_max - fp_min)

    cmap = plt.get_cmap("plasma")
    colors = cmap(normalized_probs)[:, :3]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(filtered_points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5.0)
    o3d.visualization.draw_geometries([pcd, axes])

def show_radar_pcl(cloud, intensity_threshold_percent=0.0):
    min_intensity = np.min(cloud[:, 3])
    max_intensity = np.max(cloud[:, 3])
    normalized_intensities = (cloud[:, 3] - min_intensity) / (max_intensity - min_intensity)
    filtered_idx = normalized_intensities >= intensity_threshold_percent / 100
    cmap = plt.get_cmap("plasma")
    colors = cmap(normalized_intensities[filtered_idx])[:, :3]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud[:, :3][filtered_idx])
    pcd.colors = o3d.utility.Vector3dVector(colors)
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3.0)
    o3d.visualization.draw_geometries([pcd, axes], "Radar Point Cloud Visualization")


def plot_poses(timestamps, poses, timestamps_interpolated, poses_interpolated, device_name='device'):
    plot_translations(poses[:, :3], poses_interpolated[:, :3], timestamps, timestamps_interpolated, f'Ground Truth vs {device_name.capitalize()}', device_name.lower())
    plot_rotations(poses[:, 3:], poses_interpolated[:, 3:], timestamps, timestamps_interpolated, f'Ground Truth vs {device_name.capitalize()}', device_name.lower())


def display_azimuth_fov_options(radar_config):
    draw_fov(radar_config.azimuth_bins)

def display_elevation_fov_options(radar_config):
    draw_fov(radar_config.elevation_bins)

def draw_fov(bins):
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(35, 21))
    num_sections = len(bins) // 2
    if num_sections < 16:
        color_map = [cm.get_cmap('inferno', num_sections)(i) for i in range(num_sections)]
    else:
        plasma_colors = cm.get_cmap('plasma', num_sections // 2)
        magma_colors = cm.get_cmap('inferno', num_sections // 2)
        color_map = [
            color
            for pair in zip(plasma_colors(range(num_sections // 2)), magma_colors(range(num_sections // 2)))
            for color in pair
        ]
    ax.plot([0, 0], [0, 1], color='red', lw=2)

    for bin_idx, left_bin_start in enumerate(bins[:num_sections]):
        left_bin_end = bins[bin_idx + 1] if bin_idx < num_sections else 0
        ax.fill_between([left_bin_start, left_bin_end], 0, 1, color=color_map[bin_idx], alpha=0.9)
        ax.fill_between([-left_bin_end, -left_bin_start], 0, 1, color=color_map[bin_idx], alpha=0.9)
        bin_center = (left_bin_start + left_bin_end) / 2
        bin_number = num_sections - 1 - bin_idx
        ax.text(bin_center, 0.5, f'{bin_number}', fontsize=9, ha='center', va='center', color='white')
        ax.text(-bin_center, 0.5, f'{bin_number}', fontsize=9, ha='center', va='center', color='white')
    ax.set_ylim(0, 1)
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_thetamin(np.degrees(bins[0]))
    ax.set_thetamax(np.degrees(bins[-1]))
    ax.grid(False)
    ax.set_yticklabels([])
    ax.xaxis.set_ticks([])
    bin_labels = np.degrees(bins)
    for bin_value, label in zip(bins, bin_labels):
        ax.text(bin_value, 1.02, f'{label:.1f}°', fontsize=7, ha='center')
    plt.show()


def plot_translations(gt, other, gt_ts, other_ts, title, label):
    plt.figure(figsize=(10, 6))
    plt.subplot(3, 1, 1)
    plt.plot(gt_ts, gt[:, 0], label='gt_x', color='r')
    plt.plot(other_ts, other[:, 0], label=f'{label}_x', linestyle='--', color='b')
    plt.title(f'{title} - Translation X')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(gt_ts, gt[:, 1], label='gt_y', color='r')
    plt.plot(other_ts, other[:, 1], label=f'{label}_y', linestyle='--', color='b')
    plt.title(f'{title} - Translation Y')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(gt_ts, gt[:, 2], label='gt_z', color='r')
    plt.plot(other_ts, other[:, 2], label=f'{label}_z', linestyle='--', color='b')
    plt.title(f'{title} - Translation Z')
    plt.legend()

    plt.tight_layout()
    plt.show()

def plot_rotations(gt, other, gt_ts, other_ts, title, label):
    plt.figure(figsize=(10, 8))
    plt.subplot(4, 1, 1)
    plt.plot(gt_ts, gt[:, 0], label='gt_rx', color='r')
    plt.plot(other_ts, other[:, 0], label=f'{label}_rx', linestyle='--', color='b')
    plt.title(f'{title} - Rotation X')
    plt.legend()

    plt.subplot(4, 1, 2)
    plt.plot(gt_ts, gt[:, 1], label='gt_ry', color='r')
    plt.plot(other_ts, other[:, 1], label=f'{label}_ry', linestyle='--', color='b')
    plt.title(f'{title} - Rotation Y')
    plt.legend()

    plt.subplot(4, 1, 3)
    plt.plot(gt_ts, gt[:, 2], label='gt_rz', color='r')
    plt.plot(other_ts, other[:, 2], label=f'{label}_rz', linestyle='--', color='b')
    plt.title(f'{title} - Rotation Z')
    plt.legend()

    plt.subplot(4, 1, 4)
    plt.plot(gt_ts, gt[:, 3], label='gt_w', color='r')
    plt.plot(other_ts, other[:, 3], label=f'{label}_w', linestyle='--', color='b')
    plt.title(f'{title} - Rotation W')
    plt.legend()

    plt.tight_layout()
    plt.show()
