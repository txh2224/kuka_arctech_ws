#!/usr/bin/env bash
set -eo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  source /opt/ros/humble/setup.bash
fi

exec colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
