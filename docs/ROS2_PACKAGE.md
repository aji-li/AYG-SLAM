# AYG-SLAM ROS 2 Humble package

The repository root is an `ament_cmake` package named `ayg_slam_ros2`.
The SLAM, ALIKED, YOLO/MOT, and point-cloud algorithms are still built from
their original sources; the ROS 2 layer adds installation and launch support.

## Build

Place or symlink this repository below a ROS 2 workspace `src` directory, then
build it with the existing Ubuntu 22.04 dependency environment:

```bash
source /opt/ros/humble/setup.bash
source /path/to/AYG-SLAM/scripts/setup_humble.sh
cd /path/to/ayg_slam_ros2_ws
colcon build --symlink-install --packages-select ayg_slam_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`Thirdparty/libtorch`, Pangolin, ONNX Runtime, DBoW2, g2o, CUDA 12.1, and the
model files under `models/` must exist before building. The package installs
its launch files, YAML files, RViz2 files, vocabulary, model weights, and the
small vendored runtime libraries. LibTorch stays in the source dependency
directory to avoid duplicating several gigabytes in every install space.

## Launch TUM

```bash
ros2 launch ayg_slam_ros2 ayg_slam.launch.py \
  dataset_dir:=/path/to/dataset \
  association_file:=/path/to/dataset
```

## Launch AirSim

```bash
ros2 launch ayg_slam_ros2 airsim.launch.py
```

Set `use_rviz:=false` for headless execution or `use_octomap:=false` when only
the SLAM trajectory is needed. Trajectories are written to
`~/.ros/ayg_slam` by default.
