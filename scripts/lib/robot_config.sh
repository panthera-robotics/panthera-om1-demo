#!/bin/bash
# Panthera robot config — sourced by all scripts to centralize per-robot paths.
# Usage:
#   ROBOT=go2 ./01_start_demo.sh
#   ROBOT=g1  ./01_start_demo.sh
#
# Default: go2 (the working baseline).

ROBOT="${ROBOT:-go2}"

case "$ROBOT" in
  go2)
    PANTHERA_ROBOT_TYPE="go2"
    PANTHERA_NAV_LAUNCH_PKG="go2_sdk"
    PANTHERA_NAV_LAUNCH_FILE="slam_launch.py"
    # Lidar relay: source topic from sim → topic Nav2 expects
    PANTHERA_LIDAR_SRC="/unitree_lidar"
    PANTHERA_LIDAR_DST="/utlidar/cloud_deskewed"
    ;;
  g1)
    PANTHERA_ROBOT_TYPE="g1"
    # OM1 SDK ships an empty g1_sdk/launch dir, so we author our own G1 launch
    # in /panthera_launch/ inside the nav container, mounted from launch/.
    PANTHERA_NAV_LAUNCH_PKG=""
    PANTHERA_NAV_LAUNCH_FILE="/panthera_launch/panthera_g1_slam.launch.py"
    PANTHERA_LIDAR_SRC="/unitree_lidar"
    PANTHERA_LIDAR_DST="/utlidar/cloud_deskewed"
    ;;
  *)
    echo "ERROR: unknown ROBOT='$ROBOT'. Use go2 or g1."
    exit 1
    ;;
esac

export ROBOT
export PANTHERA_ROBOT_TYPE
export PANTHERA_NAV_LAUNCH_PKG
export PANTHERA_NAV_LAUNCH_FILE
export PANTHERA_LIDAR_SRC
export PANTHERA_LIDAR_DST
