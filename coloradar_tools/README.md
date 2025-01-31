# Coloradar C Tools

## Standard Build

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

## ROS 2 Build (needs revision)
#### 1. Install `ROS2` and `colcon`

#### 2. Install dependencies
```bash
apt install ros-<version>-octomap
apt install ros-<version>-octomap-ros
```

#### 3. Create a Python environment for demo:
```bash
virtualenv -p python3 ./venv
source ./venv/bin/activate
touch ./venv/COLCON_IGNORE 
pip install catkin_pkg rosbags lark empy==3.3.4
pip install numpy open3d-python matplotlib
```

#### 4. Check OpenCV Path
In `src/CMakeLists.txt`, check the path to the library here
```text
set(OpenCV_INCLUDE_DIRS "/usr/include/opencv4")
```

To list your paths and libraries, use
```bash
pkg-config --cflags opencv4
pkg-config --libs opencv4
```

#### 5. Build
```bash
colcon build
```
Source the workspace:
```bash
source install/setup.bash
```

#### 6. Run tests
```bash
colcon test
```
Verbose testing:
```bash
colcon test --event-handler console_direct+
```

## Utils

### Requirements
- Docker 
- `apt install yq`

### ROS 1 Noetic + Cuda Docker Container
1. Edit volumes in coloradar_plus_processing_tools/create_noetic_cuda_container.yml
2. Run script
```bash
chmod +x coloradar_plus_processing_tools/build_container.sh
./coloradar_plus_processing_tools/build_container.sh
```

