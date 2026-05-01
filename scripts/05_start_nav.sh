#!/bin/bash
# Panthera demo — start SLAM + Nav2 + frontier_explorer
#
# What this brings up:
#   - openmindagi/om1_ros2_sdk:v1.0.1 container (Humble) running slam_launch.py
#     which actually launches BOTH SLAM and the full Nav2 stack
#   - A topic relay: /unitree_lidar → /utlidar/cloud_deskewed
#     (Isaac Sim publishes the former; Nav2's local_costmap subscribes to the
#     latter — the Unitree real-hardware topic name. Without this relay the
#     costmap starves and Nav2 aborts every goal.)
#
# Prerequisites:
#   01_start_demo.sh must have run first (Isaac Sim publishing /scan, /odom, /tf)
#
# After this:
#   /map         - SLAM-built occupancy grid
#   /plan        - Nav2 global plan
#   /cmd_vel_nav - Nav2 controller output (raw)
#   /cmd_vel     - Smoothed cmd_vel for the walking policy
#
# Then send goals: ./06_send_goal.sh 2 0

set -e

# Source robot config (sets PANTHERA_NAV_LAUNCH_PKG, PANTHERA_NAV_LAUNCH_FILE, etc.)
source "$(dirname "$0")/lib/robot_config.sh"

REPO_DIR="$HOME/panthera/panthera-om1-demo"
HOST_DISPLAY="${DISPLAY:-:0}"
DESKTOP_USER="${DESKTOP_USER:-user}"
XAUTH_FILE="/home/${DESKTOP_USER}/.Xauthority"

echo "==============================================="
echo " Panthera demo — start SLAM + Nav2"
echo "==============================================="
echo ""

# ============================================================
# Pre-flight
# ============================================================
if ! docker ps --format '{{.Names}}' | grep -q '^panthera_om1_demo$'; then
    echo "ERROR: panthera_om1_demo not running. Run ./01_start_demo.sh first."
    exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -q '^cmd_vel_publisher$'; then
    echo "ERROR: cmd_vel_publisher not running. Run ./01_start_demo.sh first."
    exit 1
fi

echo "[1/7] Containers from 01_start_demo.sh are up."

# ============================================================
# Verify sensor topics from sim are flowing
# ============================================================
echo ""
echo "[2/7] Verifying sensor topics from Isaac Sim..."
docker exec cmd_vel_publisher bash -c '
  source /opt/ros/jazzy/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=42
  for topic in /scan /odom /unitree_lidar /tf /tf_static /clock; do
    if timeout 3 ros2 topic info "$topic" >/dev/null 2>&1; then
      echo "      ✓ $topic"
    else
      echo "      ✗ $topic MISSING"
    fi
  done
' 2>&1 | grep -vE "^\[WARN\]|^WARNING:"

# ============================================================
# Install topic_tools in publisher container if missing
# ============================================================
echo ""
echo "[3/7] Ensuring ros-jazzy-topic-tools is installed in publisher..."
HAS_TT=$(docker exec cmd_vel_publisher bash -c '
  source /opt/ros/jazzy/setup.bash
  ros2 pkg list 2>/dev/null | grep -x topic_tools | head -1
')

if [ -z "$HAS_TT" ]; then
    echo "      Installing ros-jazzy-topic-tools..."
    docker exec cmd_vel_publisher bash -c '
      apt-get update -qq >/dev/null 2>&1
      apt-get install -y ros-jazzy-topic-tools >/dev/null 2>&1
    '
    echo "      ✓ topic_tools installed"
else
    echo "      ✓ topic_tools already present"
fi

# ============================================================
# Start lidar relay (BEFORE launching nav stack)
# ============================================================
echo ""
echo "[4/7] Starting /unitree_lidar → /utlidar/cloud_deskewed relay..."

# Kill any existing relay first to avoid duplicates
docker exec cmd_vel_publisher bash -c '
  pkill -f "topic_tools relay /unitree_lidar" 2>/dev/null
  sleep 1
' || true

docker exec -d cmd_vel_publisher bash -c '
  source /opt/ros/jazzy/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID=42
  exec ros2 run topic_tools relay /unitree_lidar /utlidar/cloud_deskewed > /tmp/relay.log 2>&1
'
sleep 4

# Verify relay is alive AND publishing
RELAY_PID=$(docker exec cmd_vel_publisher bash -c \
  'ps -ef | grep "topic_tools relay /unitree_lidar /utlidar/cloud_deskewed" | grep -v grep | awk "{print \$2}" | head -1')

if [ -z "$RELAY_PID" ]; then
    echo "      ✗ Relay failed to start. Log:"
    docker exec cmd_vel_publisher cat /tmp/relay.log 2>&1 | head -10
    exit 1
fi

RELAY_RATE=$(docker exec cmd_vel_publisher bash -c '
  source /opt/ros/jazzy/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=42
  timeout 3 ros2 topic hz /utlidar/cloud_deskewed 2>&1 | grep "average rate" | head -1
')

if [ -z "$RELAY_RATE" ]; then
    echo "      ⚠ Relay running (PID $RELAY_PID) but /utlidar/cloud_deskewed shows no rate yet."
    echo "        Continuing — costmap may catch up after launch."
else
    echo "      ✓ Relay alive (PID $RELAY_PID), $RELAY_RATE"
fi

# ============================================================
# Stop any prior nav container, start fresh
# ============================================================
echo ""
echo "[5/7] Starting panthera_nav container..."
docker rm -f panthera_nav 2>/dev/null > /dev/null || true

docker run -d --name panthera_nav \
  --network host \
  --ipc host \
  -e ROS_DISTRO=humble \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID=42 \
  -e DISPLAY="$HOST_DISPLAY" \
  -e XAUTHORITY=/tmp/.docker.xauth \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$XAUTH_FILE":/tmp/.docker.xauth:ro \
  -v "$REPO_DIR/maps:/app/om1_ros2_sdk/maps:rw" \
  -v "$REPO_DIR/launch:/panthera_launch:ro" \
  --entrypoint bash \
  openmindagi/om1_ros2_sdk:v1.0.1 \
  -c "tail -f /dev/null" > /dev/null
sleep 2
echo "      ✓ panthera_nav container up"

# ============================================================
# Launch the SLAM + Nav2 stack
# ============================================================
echo ""
echo "[6/7] Launching slam_launch.py with use_sim:=true..."
mkdir -p "$REPO_DIR/maps"
docker exec -d panthera_nav bash -c '
  source /opt/ros/humble/setup.bash
  source /app/om1_ros2_sdk/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID=42
  if [ -n "'"'$PANTHERA_NAV_LAUNCH_PKG'"'" ]; then
    exec ros2 launch '"'$PANTHERA_NAV_LAUNCH_PKG'"' '"'$PANTHERA_NAV_LAUNCH_FILE'"' use_sim:=true > /tmp/nav.log 2>&1
  else
    exec ros2 launch '"'$PANTHERA_NAV_LAUNCH_FILE'"' use_sim:=true > /tmp/nav.log 2>&1
  fi
'

echo "      Waiting 20 sec for Nav2 lifecycle to come up..."
sleep 20

# ============================================================
# Verify
# ============================================================
echo ""
echo "[7/7] Verifying nav stack is healthy..."

# Check for fatal errors first
ERROR_COUNT=$(docker exec panthera_nav bash -c '
  grep -ciE "fatal|cannot find|no such file|traceback|abort" /tmp/nav.log 2>/dev/null
' || echo "0")

# Look for the marker that means controllers came up
HAS_LIFECYCLE=$(docker exec panthera_nav bash -c '
  grep -c "Configuring" /tmp/nav.log 2>/dev/null
' || echo "0")

# Check that no buffer-stale warnings are accumulating
RECENT_STALE=$(docker exec panthera_nav bash -c '
  tail -50 /tmp/nav.log 2>/dev/null | grep -c "buffer updated in"
' || echo "0")

echo "      Lifecycle Configuring events: $HAS_LIFECYCLE"
echo "      Buffer-stale warnings in last 50 lines: $RECENT_STALE"
if [ "$RECENT_STALE" -gt "10" ]; then
    echo "      ⚠ Many stale-buffer warnings — costmap may not be ingesting lidar."
    echo "        Check relay: docker exec cmd_vel_publisher ps -ef | grep topic_tools"
fi
if [ "$HAS_LIFECYCLE" -lt "5" ]; then
    echo "      ⚠ Few lifecycle events — Nav2 may not have come up cleanly."
fi

echo ""
echo "==============================================="
echo " Nav stack started."
echo "==============================================="
echo ""
echo " Next steps:"
echo "   1. Drive Go2 to build a map:"
echo "        ./02_walk.sh circle  (then stop after ~30 sec)"
echo ""
echo "   2. View the map building in RViz:"
echo "        ./04_view_sensors.sh"
echo "        (Add a Map display, set topic to /map)"
echo ""
echo "   3. Send autonomous goal:"
echo "        ./06_send_goal.sh 2 0"
echo ""
echo "   4. Status / diagnostics:"
echo "        ./06_send_goal.sh status"
echo ""
