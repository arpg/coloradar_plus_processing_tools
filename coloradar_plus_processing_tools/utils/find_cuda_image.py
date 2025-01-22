import requests
import sys
import re


def validate_version_input(version):
    """
    Validate that the input version is in the correct X.Y format with numeric values.
    """
    if not re.match(r"^\d+\.\d+$", version):
        print("Error: CUDA version must be in the format 'X.Y' where X and Y are numeric.")
        sys.exit(1)

    major, minor = version.split(".")
    if not (major.isdigit() and minor.isdigit()):
        print("Error: CUDA version must contain only numeric values for X and Y.")
        sys.exit(1)

    return version


def get_latest_cuda_image(version):
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

            # Find the latest matching tag
            for result in data["results"]:
                tag = result["name"]
                if tag.startswith(version) and "-base-" in tag and "-ubuntu20.04" in tag:
                    match = re.match(rf"{version}\.(\d+)-base-ubuntu20.04", tag)
                    if match:
                        patch_version = int(match.group(1))
                        if patch_version > latest_patch_version[2]:  # Compare patch version
                            latest_patch = tag
                            latest_patch_version = tuple(map(int, version.split("."))) + (patch_version,)

            # Check if there are more pages
            base_url = data["next"]
    except Exception as e:
        print(f"Error fetching CUDA tags: {e}")
        sys.exit(1)

    if latest_patch:
        return latest_patch
    else:
        print(f"No matching CUDA image found for version {version}")
        sys.exit(1)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 fetch_latest_cuda.py <CUDA_VERSION>")
        sys.exit(1)

    # Validate the input version
    version = validate_version_input(sys.argv[1])

    # Get the latest CUDA image
    latest_image = get_latest_cuda_image(version)
    print(latest_image)
