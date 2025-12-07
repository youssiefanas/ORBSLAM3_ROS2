# ORBSLAM3_ROS2 Wrapper

This repository is a ROS 2 wrapper for [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3).

## Prerequisites

- **OS**: Ubuntu 22.04 or 24.04
- **ROS 2**: Rolling (Kilted) or Jazzy
- **OpenCV**: 4.x

### Build ORB_SLAM3
You need to have ORB_SLAM3 installed. You can use the original repository or a fork compatible with your system.

## How to build

1. **Clone the repository** into your ROS 2 workspace:
   ```bash
   mkdir -p colcon_ws/src
   cd colcon_ws/src
   git clone git@github.com:youssiefanas/ORBSLAM3_ROS2.git
   ```

2. **Configure Paths**:
   - Update `CMakeLists.txt`: Set the path to your Python site-packages if necessary.
   - Update `CMakeModules/FindORB_SLAM3.cmake`: Set the correct path to your `ORB_SLAM3` installation.

3. **Build**:
   ```bash
   cd ~/colcon_ws
   colcon build --symlink-install --packages-select orbslam3
   ```

## Troubleshootings
1. If you cannot find `sophus/se3.hpp`:  
Go to your `ORB_SLAM3_ROOT_DIR` and install sophus library.
```
$ cd ~/{ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus/build
$ sudo make install
```

2. Please compile with `OpenCV 4.2.0` version.
Refer this [#issue](https://github.com/zang09/ORB_SLAM3_ROS2/issues/2#issuecomment-1251850857)

## How to use

### Monocular Inertial Node

This wrapper provides a launch file `monocular_inertial_launch.py` to easily run the Monocular Inertial node.

#### Running the Launch File

```bash
ros2 launch orbslam3 monocular_inertial_launch.py \
    vocabulary_path:=/path/to/ORBvoc.txt \
    config_path:=/path/to/config.yaml \
    image_topic:=/camera/image_raw \
    imu_topic:=/imu/data
```

#### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `vocabulary_path` | `src/ORBSLAM3_ROS2/vocabulary/ORBvoc.txt` | Path to the ORB_SLAM3 vocabulary file. |
| `config_path` | *Hardware specific* | **REQUIRED**. Path to the configuration YAML file (e.g., `EuRoC.yaml` or `aqua.yaml`). |
| `image_topic` | `/camera/image_raw` | Topic name for the camera images. |
| `imu_topic` | `/rtimulib_node/imu` | Topic name for the IMU data. |
| `use_compressed` | `false` | Set to `true` if using compressed image transport. |

#### Important Paths Configuration

1.  **ORB_SLAM3 Library**: Ensure the `FindORB_SLAM3.cmake` file points to where you built the ORB_SLAM3 library.
2.  **Vocabulary File**: The launch file defaults to looking for `ORBvoc.txt` in the package share directory. You can specify a different path using the `vocabulary_path` argument.
3.  **YAML Configuration**:
    - You **must** provide a valid YAML configuration file designed for ORB-SLAM3 Monocular Inertial mode.
    - Example files can be found in the `Examples` folder of the original ORB_SLAM3 repository (e.g., `EuRoC.yaml`, `TUM-VI.yaml`).
    - Pass the absolute path to this file using the `config_path` argument.
    - **Note**: The default value in `monocular_inertial_launch.py` might point to a specific user directory (`/home/anas/...`). **You should always override this argument with your own config path.**

### Example Command

```bash
ros2 launch orbslam3 monocular_inertial_launch.py \
    config_path:=/home/user/ORB_SLAM3/Examples/Monocular-Inertial/EuRoC.yaml \
    image_topic:=/cam0/image_raw \
    imu_topic:=/imu0
```
