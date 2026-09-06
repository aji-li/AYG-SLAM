# 公开范围 / Public scope

本仓库从完整 AYG-SLAM 工作目录按文件白名单复制，使用独立 Git 历史。
当前 README 采用完整项目的中英文说明，并增加公开范围和视频入口。

This repository is an allowlisted copy of the AYG-SLAM working tree with an
independent Git history. The READMEs follow the full project's bilingual
documentation with added public-scope and video links.

| 公开内容 / Included | 用途与调整 / Purpose and adaptations |
| --- | --- |
| `Examples/RGB-D/*.cc` | RGB-D 数据输入、计时、轨迹输出；RGB-D entry points |
| `Examples/Stereo/*.cc` | KITTI / EuRoC 双目入口；stereo dataset entry points |
| `launch/*.py` | ROS 2 启动编排，本机路径已替换；orchestration with generic paths |
| `scripts/*.sh`、`build*.sh` | 环境、构建和数据集进程管理；environment/build/process management |
| `Examples/**/*.yaml` | 相机标定和模型入口片段；camera/model settings excerpts |
| `Examples/RGB-D/*.rviz` | 显示配置；visualization presets |
| `Examples/pt_to_onnx.py` | 接收用户模型路径的导出工具；export helper accepting a model argument |
| `tools/`、`demo/` | 评估工具与合成数据示例；evaluation tool and synthetic fixtures |
| `media/`、`docs/DEMO.md` | 实际数据集录屏和运行说明；actual recording and run notes |

核心跟踪、特征提取与匹配、动态过滤、建图算法及其头文件仍未公开。
模型权重、数据集、完整参数、研究计划、原始实验日志、真实轨迹文件、二进制
程序以及原 Git 历史均未复制。视频展示运行结果，不携带源码或调试日志。

Core tracking, feature extraction/matching, dynamic filtering, mapping code and
headers remain private. Weights, datasets, complete tuning, research plans, raw
logs, measured trajectory files, executables, and original Git history are omitted.
The video shows application output without source code or debug logs.

README 中提到的完整参数指南、ALIKED 研究计划和旧环境说明不在公开范围内。
相关链接因此指向本页。公开 YAML 不含动态过滤阈值和特征融合调参。

The complete parameter guide, ALIKED research plan, and older environment notes
referenced by the original README are omitted; those links lead here. Public YAML
files exclude dynamic-filter thresholds and feature-fusion tuning.

完整系统的编译和数据集运行需要私有核心、完整配置和模型。仅评估演示可在此
仓库独立运行，参见 [EVALUATION_DEMO.md](EVALUATION_DEMO.md)。

Full-system builds and dataset runs require the private core, complete settings,
and models. The evaluation demo runs independently; see
[EVALUATION_DEMO.md](EVALUATION_DEMO.md).
