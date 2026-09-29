#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /opt/ros2_ws/install/setup.bash

mode=${1:-detector}
shift || true

case "$mode" in
  detector)
    exec ros2 launch lidar_obstacle_detector detector.launch.py "$@"
    ;;
  play)
    bag=$1
    shift || true
    exec ros2 launch lidar_obstacle_detector detector.launch.py bag:="$bag" "$@"
    ;;
  offline)
    bag=$1
    shift || true
    exec python3 /opt/detector/tools/run_bag.py "$bag" --params /opt/detector/config/params.yaml "$@"
    ;;
  *)
    exec "$mode" "$@"
    ;;
esac
