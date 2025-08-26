#!/usr/bin/env bash
set -e
: "${ROS_DISTRO:=humble}"
source /opt/ros/$ROS_DISTRO/setup.bash
if [ ! -d "/home/ros/ws/build" ]; then
  cd /home/ros/ws
  colcon build --symlink-install
fi
source /home/ros/ws/install/setup.bash
exec "$@"
