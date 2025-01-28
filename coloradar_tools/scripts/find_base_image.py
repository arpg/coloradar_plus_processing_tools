import argparse
import re
import requests
import sys
import yaml


ROS_VERSIONS_FILE = 'docker/ros-versions.yaml'


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


def determine_base_image(cuda_version, ros_version_config):
    if cuda_version is None:
        return f'ubuntu:{ros_version_config["ubuntu"]}'
    image_postfix = f'-base-ubuntu{ros_version_config["ubuntu"]}'
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


def parse_ros_config(yaml_file):
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)
    return {None if k == 'default' else k: v for k, v in data['ros_versions'].items()}


def main(cuda_version, ros_version):
    ros_config = parse_ros_config(ROS_VERSIONS_FILE)
    ros_version = ros_version or None
    if ros_version not in ros_config:
        print(f"Error: ROS version must be one of the following: {tuple(ros_config.keys())}, not {ros_version}")
        sys.exit(1)

    version_config = ros_config[ros_version]
    cuda_version = validate_cuda_version(cuda_version)
    base_image = determine_base_image(cuda_version, version_config)
    return base_image


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Find base Ubuntu image.")
    parser.add_argument("--cuda", required=False, help="CUDA version in X.Y format (e.g., 12.6).")
    parser.add_argument("--ros", required=False, help="ROS version (e.g., noetic).")
    args = parser.parse_args()
    image = main(args.cuda, args.ros)
    print(image)
