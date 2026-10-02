#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
project_root="$PWD"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source "$project_root/build-linux/uros/firmware/dev_ws/install/local_setup.bash"
cd "$project_root/build-linux/uros/firmware/mcu_ws"
colcon build --cmake-clean-cache --merge-install --parallel-workers 4 \
  --packages-ignore-regex='.*_cpp' --metas "$project_root/tools/microros-colcon.meta" \
  --cmake-args --no-warn-unused-cli -DCMAKE_POSITION_INDEPENDENT_CODE=OFF \
  -DTHIRDPARTY=ON -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTING=OFF \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="$project_root/tools/microros-toolchain.cmake"
python3 "$project_root/tools/pack-microros.py"
