#!/usr/bin/env bash
set -e

cd /colcon_ws

# the image drops apt lists after build, so installs need a fresh index
apt-get update

# xarm_gazebo and thirdparty/realsense_gazebo_plugin are real-hardware-only, so
# exclude them from the build. Create the markers here so a fresh clone (whose
# submodules won't carry untracked files) still builds. These live inside the
# vendored xarm_ros2 submodule, so recreate them on every run to stay idempotent.
touch src/xarm_ros2/xarm_gazebo/COLCON_IGNORE \
      src/xarm_ros2/thirdparty/realsense_gazebo_plugin/COLCON_IGNORE

# Ignored packages are invisible to --ignore-src, so any key
# they are referenced by must be skipped explicitly.
rosdep install --from-paths src --ignore-src --rosdistro humble -y \
  --skip-keys "xarm_gazebo realsense_gazebo_plugin"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
