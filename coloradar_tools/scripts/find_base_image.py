import argparse
import os
import re
import requests
import sys


ROS_REQUIREMENTS = {
    None: {
        'ubuntu_version': '24.04',
        'gcc_version': '12',
        'python_version': '3.11'
    },
    'noetic': {
        'ubuntu_version': '20.04',
        'gcc_version': '11',
        'python_version': '3.11'
    }
}


def validate_cuda_version(cuda_version):
    """
    Validate that the input version is in the correct X.Y format with numeric values.
    """
    if cuda_version is None:
        return None

    if not re.match(r"^\d+\.\d+$", cuda_version):
        print("Error: CUDA version must be in the format 'X.Y' where X and Y are numeric.")
        sys.exit(1)

    major, minor = cuda_version.split(".")
    if not (major.isdigit() and minor.isdigit()):
        print("Error: CUDA version must contain only numeric values for X and Y.")
        sys.exit(1)

    return cuda_version


def validate_ros_version(ros_version):
    ros_version = ros_version or None
    if ros_version not in ROS_REQUIREMENTS:
        print(f"Error: ROS version must be one of the following: {tuple(ROS_REQUIREMENTS.keys())}, not {ros_version}")
        sys.exit(1)
    return ros_version


def determine_base_image(cuda_version, ros_version):
    if cuda_version is None:
        return f'ubuntu:{ROS_REQUIREMENTS[ros_version]["ubuntu_version"]}'
    image_postfix = f'-base-ubuntu{ROS_REQUIREMENTS[ros_version]["ubuntu_version"]}'
    image_tag = get_latest_cuda_tag(cuda_version, image_postfix)
    return f'nvidia/cuda:{image_tag}'


def get_latest_cuda_tag(cuda_version, base_image_postfix):
    """
    Query Docker Hub to find the latest patch version for a given CUDA X.Y version.
    """
    base_url = "https://hub.docker.com/v2/repositories/nvidia/cuda/tags"
    params = {"page_size": 100}  # Fetch 100 tags at a time
    latest_patch = None
    latest_patch_version = (0, 0, 0)  # Default to lowest version tuple

    try:
        while base_url:
            response = requests.get(base_url, params=params)
            response.raise_for_status()
            data = response.json()
            for result in data["results"]:
                tag = result["name"]
                if tag.startswith(cuda_version) and base_image_postfix in tag:
                    match = re.match(rf"{cuda_version}\.(\d+){base_image_postfix}", tag)
                    if match:
                        patch_version = int(match.group(1))
                        if patch_version > latest_patch_version[2]:  # Compare patch version
                            latest_patch = tag
                            latest_patch_version = tuple(map(int, cuda_version.split("."))) + (patch_version,)

            # Check if there are more pages
            base_url = data["next"]
    except Exception as e:
        print(f"Error fetching CUDA tags: {e}")
        sys.exit(1)

    if latest_patch:
        return latest_patch
    else:
        print(f"No matching CUDA image found for version {cuda_version}")
        sys.exit(1)


def export_settings(**env_dict):
    for key, value in env_dict.items():
        env_name = f'DOCKER_{key.upper()}'
        # os.environ[env_name] = value
        print(f'export {env_name}={value}')


def main(cuda_version, ros_version):
    cuda_version = validate_cuda_version(cuda_version)
    ros_version = validate_ros_version(ros_version)
    base_image = determine_base_image(cuda_version, ros_version)
    export_settings(base_image=base_image, ros_version=ros_version, cuda_version=cuda_version, **ROS_REQUIREMENTS[ros_version])
    return base_image


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Find base Ubuntu image.")
    parser.add_argument("--cuda", required=False, help="CUDA version in X.Y format (e.g., 12.6).")
    parser.add_argument("--ros", required=False, help="ROS version (e.g., noetic).")
    args = parser.parse_args()
    image = main(args.cuda, args.ros)
