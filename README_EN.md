# AYG-SLAM: SLAM System Based on Learned Features and Dynamic Feature Removal

<p align="center">
  <a href="README.md">简体中文</a> | <strong>English</strong>
</p>

## Recorded Demo

### AirSim Sequence 3: OctoMap Mapping

[![OctoMap construction on AirSim Sequence 3](media/airsim-sequence3-octomap-preview.gif)](media/airsim-sequence3-octomap.mp4)

[Watch / download the AirSim mapping video](media/airsim-sequence3-octomap.mp4)

### TUM walking_xyz: Feature Tracking

[![Actual TUM walking_xyz run](media/tum-walking-xyz-preview.gif)](media/tum-walking-xyz.mp4)

[Watch / download the recording](media/tum-walking-xyz.mp4) · [Standalone evaluation example](docs/EVALUATION_DEMO.md)

AYG-SLAM extends ORB-SLAM2 with ALIKED frontend features, dynamic object detection
and tracking, and geometric consistency checks. It supports RGB-D and stereo
workflows for camera trajectory estimation and static environment mapping, with
ROS 2 point-cloud publication, OctoMap integration, and RViz2 visualization.

Dataset examples cover TUM RGB-D, AirSim RGB-D recordings, KITTI stereo, and EuRoC stereo.

<p align="center">
  <a href="#features">Features</a> ·
  <a href="#requirements">Requirements</a> ·
  <a href="#build">Build</a> ·
  <a href="#run">Run</a> ·
  <a href="#configuration-and-models">Configuration and Models</a> ·
  <a href="#documentation">Documentation</a> ·
  <a href="#acknowledgements">Acknowledgements</a>
</p>

## Features

- **Learned feature frontend:** ALIKED keypoints and descriptors integrated into
  the ORB-SLAM2 tracking and local mapping pipeline.
- **Loop closure and relocalization:** Place recognition, loop detection, and
  recovery after tracking loss.
- **Dynamic feature filtering:** YOLO detection or segmentation combined with
  epipolar and depth consistency checks.
- **Multi-object tracking:** motcpp backends for associating detections across
  frames, with configurable dynamic classes such as `person` and `uav`.
- **Static mapping:** filtered local point clouds published as ROS 2
  `sensor_msgs/PointCloud2` for OctoMap construction.
- **Visualization:** Pangolin SLAM viewer and RViz2 displays for point clouds,
  occupancy maps, TF, dynamic 3D boxes, and UAV meshes.

### Code Guide

| Module | Main responsibilities |
| --- | --- |
| [ALIKEDextractor](src/ALIKEDextractor.cc) | Model loading, inference-device selection, and runtime wrapper |
| [System](src/System.cc) | Image input interfaces, thread cleanup, and trajectory export |
| [Frame](src/Frame.cc), [KeyFrame](src/KeyFrame.cc), [MapPoint](src/MapPoint.cc) | Frame poses, keyframe covisibility, and map-point observations |
| [YoloDetector](src/YoloDetector.cc), [YoloSegDetector](src/YoloSegDetector.cc) | Detection/segmentation inference interfaces and track association |
| [PointCloudMapping](src/PointCloudMapping.cc) | Keyframe queues, coordinate transforms, ROS point clouds, and markers |
| [FrameDrawer](src/FrameDrawer.cc), [MapDrawer](src/MapDrawer.cc), [Viewer](src/Viewer.cc) | Image overlays, map rendering, and interactive display |

## Requirements

The current build scripts target **Ubuntu 22.04 and ROS 2 Humble**.

| Component | Requirement / current configuration |
| --- | --- |
| Compiler | GCC / G++ 11; scripts default to `gcc-11` and `g++-11` |
| Build tools | CMake 3.26 or newer; Ninja by default |
| Computer vision | OpenCV 4.x, Eigen 3, Pangolin |
| C++ libraries | yaml-cpp, Boost, bundled DBoW2 and g2o |
| ALIKED inference | LibTorch and CUDA Toolkit 12.1 or newer |
| YOLO inference | ONNX Runtime GPU 1.18.0 |
| ROS integration | ROS 2 Humble and `ament_cmake` |
| Mapping and visualization | PCL, OctoMap, ROS 2 `octomap_server`, RViz2 |
| Python utilities | Python 3.10; packages in [env/requirements.txt](env/requirements.txt) |

The build expects these dependencies to be available locally:

```text
Thirdparty/libtorch/
Thirdparty/Pangolin/install/
Thirdparty/YOLOs-CPP/onnxruntime-linux-x64-gpu-1.18.0/
```

Prepare these libraries and the model weights before building; a source checkout
alone may not contain the large binary dependencies. Use LibTorch and ONNX
Runtime builds compatible with your CUDA environment.

[scripts/setup_humble.sh](scripts/setup_humble.sh) configures the local library
paths and defaults to `/usr/local/cuda-12.1`. Adjust that script if your dependency
locations differ. The current motcpp build sets `MOTCPP_ENABLE_ONNX=OFF`, disabling
its ONNX-based ReID backend.

## Build

Run the following commands from the repository root after installing the
required dependencies:

```bash
source scripts/setup_humble.sh
```

Both build modes require the ROS 2 Humble environment because the root project
uses `ament_cmake`. The scripts build DBoW2 and g2o and extract
`Vocabulary/ORBvoc.txt.tar.gz` automatically.

### RGB-D SLAM

```bash
./build.sh
```

This uses `build_native/` and produces `Examples/RGB-D/rgbd_tum`.

### RGB-D SLAM with OctoMap

```bash
./build_octomap.sh
```

This uses `build_humble/` and produces `Examples/RGB-D/rgbd_tum_octomap`.

To also build the stereo OctoMap examples in the same build directory:

```bash
cmake --build build_humble --parallel 4 \
  --target stereo_kitti_octomap stereo_euroc_octomap
```

The scripts accept `BUILD_DIR`, `JOBS`, `CMAKE_BIN`, and
`ORB_SLAM2_OPENCV_DIR` overrides. For example, to use system CMake and limit
parallel compilation:

```bash
CMAKE_BIN="$(command -v cmake)" JOBS=4 ./build_octomap.sh
```

Build the two modes sequentially: they share third-party build directories and
source-tree outputs such as `lib/libORB_SLAM2.so`.

For a ROS 2 workspace build and installed launch files, see
[ROS 2 package instructions](docs/ROS2_PACKAGE.md).

## Run

Run these examples from the repository root. Replace `/path/to/...` with your
own dataset paths; the wrapper scripts contain machine-specific defaults, so
explicit arguments are recommended.

### Prepare RGB-D data

TUM and AirSim RGB-D runs take four arguments: vocabulary, camera settings,
sequence directory, and an RGB/depth association file. Each association line
contains:

```text
rgb_timestamp rgb/image.png depth_timestamp depth/image.png
```

Image paths are relative to the sequence directory. Set the camera calibration,
depth scale, model paths, and dynamic classes in the dataset YAML before running.

### TUM RGB-D

For trajectory estimation with the normal build:

```bash
source scripts/setup_humble.sh
./Examples/RGB-D/rgbd_tum \
  Vocabulary/ORBvoc.txt \
  Examples/RGB-D/TUM3.yaml \
  /path/to/tum_sequence \
  /path/to/associations.txt
```

For the OctoMap pipeline, after building the OctoMap variant:

```bash
./scripts/run_octomap_tum.sh \
  Vocabulary/ORBvoc.txt \
  Examples/RGB-D/TUM3.yaml \
  /path/to/tum_sequence \
  /path/to/associations.txt
```

### AirSim RGB-D

```bash
./scripts/run_octomap_airsim.sh \
  Vocabulary/ORBvoc.txt \
  Examples/RGB-D/airsim_new.yaml \
  /path/to/airsim_sequence \
  /path/to/associations.txt
```

AirSim recordings must use the RGB-D association format above and a camera/depth
configuration matching the recording.

### KITTI stereo

The sequence directory must contain `image_0/`, `image_1/`, and `times.txt`.
Choose the YAML that matches the sequence calibration.

```bash
./scripts/run_octomap_kitti.sh \
  Vocabulary/ORBvoc.txt \
  Examples/Stereo/KITTI00-02.yaml \
  /path/to/kitti/sequences/00
```

### EuRoC stereo

```bash
./scripts/run_octomap_euroc.sh \
  Vocabulary/ORBvoc.txt \
  Examples/Stereo/EuRoC.yaml \
  /path/to/MH_01_easy/mav0/cam0/data \
  /path/to/MH_01_easy/mav0/cam1/data \
  Examples/Stereo/EuRoC_TimeStamps/MH01.txt
```

### ROS 2 mapping and outputs

The OctoMap wrappers source the Humble environment, start `octomap_server`,
optionally start RViz2, and run the SLAM executable. The mapping data flow is:

```text
RGB-D / stereo images → AYG-SLAM → /AYG/Local_Point_Clouds
                                          ↓
                                    octomap_server
                                          ↓
                                   /octomap_binary → RViz2
```

Set `START_RVIZ=0` before a wrapper command to disable RViz2. This controls RViz2
only; configure the SLAM viewer separately if needed.

RGB-D runs write `CameraTrajectory.txt` and `KeyFrameTrajectory.txt` in the
working directory. Installed ROS 2 launch
files use `~/.ros/ayg_slam` as the default output directory. Generated trajectories
and maps are runtime outputs.

## Configuration and Models

| Workflow | Settings | YOLO model expected by the settings |
| --- | --- | --- |
| TUM RGB-D | [TUM3.yaml](Examples/RGB-D/TUM3.yaml) | `models/yolo26n-seg.onnx` |
| AirSim RGB-D | [airsim_new.yaml](Examples/RGB-D/airsim_new.yaml) | `models/yolo26n_uav.onnx` |
| KITTI stereo | [KITTI00-02.yaml](Examples/Stereo/KITTI00-02.yaml), [KITTI03.yaml](Examples/Stereo/KITTI03.yaml), [KITTI04-12.yaml](Examples/Stereo/KITTI04-12.yaml) | Check the selected YAML |
| EuRoC stereo | [EuRoC.yaml](Examples/Stereo/EuRoC.yaml) | Check the selected YAML |

ALIKED loads weights from `models/<model-name>.pt`; the RGB-D configurations
currently select `aliked-n32`. Ensure the selected ALIKED weights, YOLO ONNX model,
and label file exist before running. Custom model weights must match your label
file and detector configuration.

RViz2 presets are available for
[TUM](Examples/RGB-D/octomap_view_tum.rviz) and
[AirSim](Examples/RGB-D/octomap_view_airsim.rviz).

[Examples/pt_to_onnx.py](Examples/pt_to_onnx.py) provides a YOLO export utility.
Its optional Python dependencies can be installed with:

```bash
python3 -m pip install -r env/requirements.txt
```

These Python packages do not install the C++ / CUDA / ROS dependencies.

## Repository Layout

```text
AYG-SLAM/
├── src/                 # SLAM, ALIKED, YOLO/MOT, and mapping implementation
├── include/             # C++ headers
├── Examples/            # Dataset executables, camera settings, and RViz presets
├── launch/              # ROS 2 launch files
├── scripts/             # Build, environment, run, and evaluation helpers
├── Thirdparty/          # Third-party sources and local runtime dependencies
├── Vocabulary/          # ORB bag-of-words vocabulary
├── models/              # Local ALIKED and YOLO weights and labels
├── 3dmod/               # UAV visualization meshes
├── env/                 # Dependency lists
└── docs/                # Setup, configuration, and integration notes
```

## Documentation

- [ROS 2 package build and launch](docs/ROS2_PACKAGE.md)
- [Original ORB-SLAM2 README](https://github.com/raulmur/ORB_SLAM2)

## Acknowledgements

We thank the authors and maintainers of the following open-source projects.
AYG-SLAM builds on their implementations and research:

| Project | Contribution to AYG-SLAM |
| --- | --- |
| [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) | Base SLAM architecture, tracking, local mapping, loop closure, and relocalization. |
| [ALIKED](https://github.com/Shiaoming/ALIKED) | Learned keypoint detection and local descriptor extraction. |
| [YOLOs-CPP](https://github.com/Geekgineer/YOLOs-CPP) | C++ YOLO detection and segmentation inference. |
| [motcpp](https://github.com/Geekgineer/motcpp) | Multi-object tracking backends used with YOLO detections. |
| [octomap_server / octomap_mapping](https://github.com/OctoMap/octomap_mapping) | ROS point-cloud integration and OctoMap occupancy mapping. |
| [Pangolin](https://github.com/stevenlovegrove/Pangolin) | Interactive SLAM visualization. |
| [g2o](https://github.com/RainerKuemmerle/g2o) | Graph optimization and bundle adjustment. |
| [DBoW2](https://github.com/dorian3d/DBoW2) | Bag-of-words place recognition for loop detection and relocalization. |

The bundled DBoW2 and g2o versions originate from the ORB-SLAM2 distribution.
Please refer to the upstream repositories for their publications and citation
instructions when using this work in research.

## License

See [LICENSE.txt](LICENSE.txt) and [License-gpl.txt](License-gpl.txt) for the
repository's existing GPLv3 license notices. Third-party components retain their
respective licenses; consult the license files in their source directories.
