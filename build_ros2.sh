#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
livox_workspace="${LIVOX_WS:-$(dirname "$workspace_dir")/livox_ros_ws}"
if [[ ! -f "$livox_workspace/install/setup.bash" ]]; then
  echo "Build livox_ros_driver2 first, then set LIVOX_WS to its workspace." >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$livox_workspace/install/setup.bash"
cd "$workspace_dir"
export MAKEFLAGS="${MAKEFLAGS:--j2}"
colcon build --symlink-install --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
