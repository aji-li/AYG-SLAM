#!/usr/bin/env bash

AYG_SLAM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
export CUDA_HOME=/usr/local/cuda-12.1
export PATH="${CUDA_HOME}/bin:${AYG_SLAM_ROOT}/Thirdparty/cmake/bin:${PATH}"
export LD_LIBRARY_PATH="${AYG_SLAM_ROOT}/lib:${AYG_SLAM_ROOT}/Thirdparty/libtorch/lib:${AYG_SLAM_ROOT}/Thirdparty/YOLOs-CPP/onnxruntime-linux-x64-gpu-1.18.0/lib:${AYG_SLAM_ROOT}/Thirdparty/Pangolin/install/lib:${CUDA_HOME}/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export CMAKE_PREFIX_PATH="${AYG_SLAM_ROOT}/Thirdparty/Pangolin/install:${AYG_SLAM_ROOT}/Thirdparty/libtorch${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
