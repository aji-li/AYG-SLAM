# 公开范围 / Public scope

本目录从完整 AYG-SLAM 工作目录中按文件白名单独立复制，使用全新的 Git
历史。原仓库的源码、提交记录和可见性不受影响。

This directory is a file-allowlisted export of the AYG-SLAM working tree with a
fresh Git history. The original repository and its visibility are unchanged.

| 公开内容 / Included | 来源与调整 / Source and adaptations |
| --- | --- |
| `examples/rgbd_tum_octomap.cc` | RGB-D 入口副本，补回上游版权头；integration excerpt with upstream notice |
| `examples/ros2/ayg_slam.launch.py` | ROS 2 启动层副本，替换本机数据路径；generic dataset default |
| `tools/summarize_evo_multirun.py` | 评估脚本副本，外置数据集信息；external sequence counts and output checks |
| `demo/create_demo_results.py` | 新增独立演示数据生成器；new synthetic fixture generator |
| README、NOTICE、许可证 | 展示说明、来源说明与原许可证文本；documentation and license |

未公开核心跟踪、特征提取与匹配、动态过滤、建图算法实现及其头文件；
未复制模型、数据集、调参配置、研究计划、实验日志、真实轨迹、二进制文件或原 Git 历史。

Core tracking, feature extraction/matching, dynamic filtering and mapping
implementations and headers are omitted, along with weights, datasets, tuned
configurations, research plans, experiment logs, measured trajectories, binaries,
and the original Git history.

RGB-D 和 ROS 2 示例供代码阅读，依赖未公开的完整系统，不能在本仓库中单独编译或启动。
评估演示只需要 Python 3 标准库，可按 README 独立运行。

The RGB-D and ROS 2 excerpts require the omitted full system and are for source
review. The evaluation demonstration runs independently with Python 3's standard
library. This repository does not publish SLAM benchmark claims.
