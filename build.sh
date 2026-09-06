#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CMAKE_BIN="${CMAKE_BIN:-${ROOT_DIR}/Thirdparty/cmake/bin/cmake}"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build_native}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="${JOBS:-$(nproc)}"
CMAKE_GENERATOR="${CMAKE_GENERATOR:-Ninja}"
OPENCV_DIR="${ORB_SLAM2_OPENCV_DIR:-/usr/lib/x86_64-linux-gnu/cmake/opencv4}"

export CC="${CC:-/usr/bin/gcc-11}"
export CXX="${CXX:-/usr/bin/g++-11}"

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "ROS 2 Humble was not found at /opt/ros/humble." >&2
  exit 1
fi

detect_cuda_arch() {
  if command -v nvidia-smi >/dev/null 2>&1; then
    local capability
    capability="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -n1 | tr -d '[:space:]')"
    if [[ -n "${capability}" && "${capability}" =~ ^[0-9]+\.[0-9]+$ ]]; then
      echo "${capability}"
      return 0
    fi
  fi
  return 1
}

CUDA_CAPABILITY="$(detect_cuda_arch || true)"
if [[ -n "${CUDA_CAPABILITY}" ]]; then
  CUDA_ARCHITECTURES_DEFAULT="${CUDA_CAPABILITY/./}"
  TORCH_CUDA_ARCH_LIST_DEFAULT="${CUDA_CAPABILITY}"
else
  CUDA_ARCHITECTURES_DEFAULT="native"
  TORCH_CUDA_ARCH_LIST_DEFAULT=""
fi

CUDA_ARCHITECTURES="${CMAKE_CUDA_ARCHITECTURES:-${CUDA_ARCHITECTURES_DEFAULT}}"
TORCH_CUDA_ARCH_LIST_VALUE="${TORCH_CUDA_ARCH_LIST:-${TORCH_CUDA_ARCH_LIST_DEFAULT}}"

echo "Using CMake: ${CMAKE_BIN}"
echo "Using C compiler: ${CC}"
echo "Using C++ compiler: ${CXX}"
echo "Build dir: ${BUILD_DIR}"
echo "OpenCV_DIR: ${OPENCV_DIR}"
echo "ENABLE_ADSG_OCTOMAP: OFF"
echo "CUDA architectures: ${CUDA_ARCHITECTURES}"
echo "TORCH_CUDA_ARCH_LIST: ${TORCH_CUDA_ARCH_LIST_VALUE:-<auto>}"

echo "Configuring and building Thirdparty/DBoW2 ..."
rm -f "${ROOT_DIR}/Thirdparty/DBoW2/build/CMakeCache.txt"
rm -rf "${ROOT_DIR}/Thirdparty/DBoW2/build/CMakeFiles"
"${CMAKE_BIN}" -Wno-dev -G "${CMAKE_GENERATOR}" -S "${ROOT_DIR}/Thirdparty/DBoW2" -B "${ROOT_DIR}/Thirdparty/DBoW2/build" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_C_COMPILER="${CC}" \
  -DCMAKE_CXX_COMPILER="${CXX}" \
  -DOpenCV_DIR="${OPENCV_DIR}"
"${CMAKE_BIN}" --build "${ROOT_DIR}/Thirdparty/DBoW2/build" -j"${JOBS}"

echo "Configuring and building Thirdparty/g2o ..."
rm -f "${ROOT_DIR}/Thirdparty/g2o/build/CMakeCache.txt"
rm -rf "${ROOT_DIR}/Thirdparty/g2o/build/CMakeFiles"
"${CMAKE_BIN}" -Wno-dev -G "${CMAKE_GENERATOR}" -S "${ROOT_DIR}/Thirdparty/g2o" -B "${ROOT_DIR}/Thirdparty/g2o/build" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_C_COMPILER="${CC}" \
  -DCMAKE_CXX_COMPILER="${CXX}"
"${CMAKE_BIN}" --build "${ROOT_DIR}/Thirdparty/g2o/build" -j"${JOBS}"

echo "Uncompress vocabulary ..."
(
  cd "${ROOT_DIR}/Vocabulary"
  tar -xf ORBvoc.txt.tar.gz
)

echo "Configuring and building ORB_SLAM2 ..."
rm -f "${BUILD_DIR}/CMakeCache.txt"
rm -rf "${BUILD_DIR}/CMakeFiles"

cmake_args=(
  -G "${CMAKE_GENERATOR}"
  -S "${ROOT_DIR}"
  -B "${BUILD_DIR}"
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
  -DORB_SLAM2_OPENCV_DIR="${OPENCV_DIR}"
  -DENABLE_ADSG_OCTOMAP=OFF
  -DAYG_SLAM_LEGACY_OUTPUT=ON
  -DCMAKE_C_COMPILER="${CC}"
  -DCMAKE_CXX_COMPILER="${CXX}"
  -DCMAKE_CUDA_HOST_COMPILER="${CXX}"
  -DCMAKE_CUDA_ARCHITECTURES="${CUDA_ARCHITECTURES}"
)

if [[ -n "${TORCH_CUDA_ARCH_LIST_VALUE}" ]]; then
  cmake_args+=(-DTORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST_VALUE}")
fi

"${CMAKE_BIN}" -Wno-dev "${cmake_args[@]}"
"${CMAKE_BIN}" --build "${BUILD_DIR}" -j"${JOBS}" --target ORB_SLAM2 rgbd_tum

echo
echo "Build completed in ${BUILD_DIR}"
echo "  - ${ROOT_DIR}/Examples/RGB-D/rgbd_tum"
