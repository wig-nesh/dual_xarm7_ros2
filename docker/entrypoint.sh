#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /colcon_ws/install/setup.bash ]; then
  source /colcon_ws/install/setup.bash
fi

exec "$@"
