# Coloradar Tools

## 1. Docker Build
### 1.1. Requirements
- Linux OS
- Docker 
- Docker Compose
- `yq`

### 2.2. Build
```bash
chmod +x build_image.sh
./build_image.sh <ros_version>
```

#### Available Image Configurations
| ros_version | OS                 | Python |
|-------------|--------------------|--------|
| noetic      | Ubuntu 20.04 Focal | 3.8    |
| humble      | Ubuntu 22.04 Jammy | 3.10   |
| jazzy       | Ubuntu 24.04 Noble | 3.12    |


### 2.3. Compose Utils
```bash
docker compose build
docker compose up <util_name>
```
Available utils:
#### 2.3.1. `add_heatmaps_to_bags`
Requirements:
1. Install **NVIDIA CUDA** locally.
2. Build a **noetic** image using `build_image.sh`.
3. Configure the **base** service in `docker-compose.yml`:
    - 2.1. Volume the local directory containing the bag files to `/root/bags`.
    - 2.2. Specify the path to the `calib` directory relatively to `/root/bags`. Defaults to `/root/bags/calib`.

   
#### Available Utils
- `add_heatmaps_to_bags`: For every bag in `/root/bags`, create a new topic `/cascade/heatmaps` and convert datacubes from `/cascade/datacubes` into heatmaps with identical metadata.
- `binarize_bags`: Convert every bag in `/root/bags` into binary files and save as `<bag_name>.zip`. 
- `export_dataset`: *TBD*.


## 2. Standard Build

#### 1. Install dependencies:
```bash
apt install libpcl-dev liboctomap-dev libgtest-dev libopencv-dev
```
Optional dependencies:
```bash
apt install pybind11-dev
```

#### 2. Build:
```bash
mkdir build
cmake -S coloradar_tools -B build
make -C build
```

#### 3. Run tests:
```bash
./build/coloradar_tests
```

#### 4. Create a Python environment:
```bash
virtualenv -p python3 ./venv
source ./venv/bin/activate
pip install numpy open3d matplotlib h5py
```

[//]: # ()
[//]: # (## ROS 2 Build &#40;needs revision&#41;)

[//]: # (#### 1. Install `ROS2` and `colcon`)

[//]: # ()
[//]: # (#### 2. Install dependencies)

[//]: # (```bash)

[//]: # (apt install ros-<version>-octomap)

[//]: # (apt install ros-<version>-octomap-ros)

[//]: # (```)

[//]: # ()
[//]: # (#### 3. Create a Python environment for demo:)

[//]: # (```bash)

[//]: # (virtualenv -p python3 ./venv)

[//]: # (source ./venv/bin/activate)

[//]: # (touch ./venv/COLCON_IGNORE )

[//]: # (pip install catkin_pkg rosbags lark empy==3.3.4)

[//]: # (pip install numpy open3d-python matplotlib)

[//]: # (```)

[//]: # ()
[//]: # (#### 4. Check OpenCV Path)

[//]: # (In `src/CMakeLists.txt`, check the path to the library here)

[//]: # (```text)

[//]: # (set&#40;OpenCV_INCLUDE_DIRS "/usr/include/opencv4"&#41;)

[//]: # (```)

[//]: # ()
[//]: # (To list your paths and libraries, use)

[//]: # (```bash)

[//]: # (pkg-config --cflags opencv4)

[//]: # (pkg-config --libs opencv4)

[//]: # (```)

[//]: # ()
[//]: # (#### 5. Build)

[//]: # (```bash)

[//]: # (colcon build)

[//]: # (```)

[//]: # (Source the workspace:)

[//]: # (```bash)

[//]: # (source install/setup.bash)

[//]: # (```)

[//]: # ()
[//]: # (#### 6. Run tests)

[//]: # (```bash)

[//]: # (colcon test)

[//]: # (```)

[//]: # (Verbose testing:)

[//]: # (```bash)

[//]: # (colcon test --event-handler console_direct+)

[//]: # (```)
