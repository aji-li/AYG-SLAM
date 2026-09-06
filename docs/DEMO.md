# 数据集运行与录制 / Dataset run and recording

[观看 / 下载完整视频 · Full video](../media/tum-walking-xyz.mp4)

![实际运行预览 / Actual run preview](../media/tum-walking-xyz-preview.gif)

## 本次运行

- 数据集：TUM RGB-D `rgbd_dataset_freiburg3_walking_xyz`。
- 录制日期：2026-09-06。
- 输入：本机关联文件中的 827 对 RGB / 深度图像；处理帧编号为 0–826。
- 输出：827 条相机位姿；轨迹和原始调试日志保留在本地，未上传。
- 退出状态：0，程序正常完成。
- 环境：Ubuntu 22.04、ROS 2 Humble、NVIDIA RTX 3090。
- 显示内容：Pangolin 相机轨迹与稀疏地图、OpenCV 跟踪特征与 YOLO 分割叠加。

录制使用完整研发目录中已构建的 RGB-D 程序，ALIKED 与 YOLO 已启用。
本次使用 `ORB_SLAM2_DISABLE_POINTCLOUD_MAPPING=1` 关闭点云发布，因此视频
展示的是 SLAM 稀疏地图，不是 OctoMap 占据地图。首轮启用点云发布时发现了
退出阶段的 PCL 点云释放崩溃；本视频来自关闭该模块后正常退出的独立重跑。
该点云清理问题尚未修复。

视频通过隔离的虚拟显示器录制，仅包含应用窗口。保留原始墙钟播放速度，
20 fps 采集；移除了退出后的黑屏，并添加标题、画面说明和署名。
README 动图是完整视频中间的 7 秒片段，降低了分辨率和帧率。
本视频用于展示运行行为，不作为精度或实时性能基准。

完整环境下的运行方式如下；公开仓库缺少核心和完整配置，不能直接执行该命令：

```bash
source scripts/setup_humble.sh
ORB_SLAM2_DISABLE_POINTCLOUD_MAPPING=1 \
  ./Examples/RGB-D/rgbd_tum_octomap \
  Vocabulary/ORBvoc.txt \
  Examples/RGB-D/TUM3.yaml \
  /path/to/rgbd_dataset_freiburg3_walking_xyz \
  /path/to/associate_ours.txt
```

## Run details

The full local development build processed all 827 associated RGB-D frames
(frame IDs 0–826), saved 827 camera poses, and exited with status 0 on 2026-09-06.
The environment used Ubuntu 22.04, ROS 2 Humble, and an NVIDIA RTX 3090.
ALIKED and YOLO were enabled.

The recording shows Pangolin's camera trajectory and sparse map alongside the
OpenCV tracking view with feature and segmentation overlays. Point-cloud
publication was explicitly disabled using
`ORB_SLAM2_DISABLE_POINTCLOUD_MAPPING=1`; this is not an OctoMap demo.
An earlier run with point-cloud publication enabled encountered a PCL cleanup
crash after saving trajectories. The published video comes from a separate,
successful run with that module disabled. The cleanup issue remains unresolved.

Only application windows on an isolated virtual display were captured, at 20 fps
and original wall-clock speed. Post-exit black frames were removed and titles,
labels, and credits were added. The README GIF is a seven-second excerpt at lower
resolution and frame rate. This demonstration is not an accuracy or real-time
performance benchmark. Raw logs and trajectory files are not published.

## 数据集署名 / Dataset attribution

The video and previews include imagery from the
[TUM RGB-D Dataset and Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset),
sequence `freiburg3_walking_xyz`.

**J. Sturm, N. Engelhard, F. Endres, W. Burgard, and D. Cremers.**
*A Benchmark for the Evaluation of RGB-D SLAM Systems.* IROS, 2012.

The dataset is provided under
[Creative Commons Attribution 4.0](https://creativecommons.org/licenses/by/4.0/),
as stated on the dataset website. This recording adds AYG-SLAM feature,
segmentation, and map visualizations, plus explanatory titles. The previews are
resized excerpts of the recording. These additions do not imply endorsement by
the dataset authors. Dataset imagery remains subject to its attribution license.
