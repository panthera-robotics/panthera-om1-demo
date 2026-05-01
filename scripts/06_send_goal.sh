#!/bin/bash
# Panthera demo — send autonomous navigation goal to Nav2
# Tells Go2 to walk autonomously to a given (x, y) coordinate in the map frame.
#
# What this handles automatically:
#   1. Installs ros-jazzy-topic-tools in cmd_vel_publisher (if missing)
#   2. Starts /unitree_lidar → /utlidar/cloud_deskewed relay (if not running)
#      Nav2's local_costmap subscribes to /utlidar/cloud_deskewed (the real-Go2
#      topic name); Isaac Sim publishes on /unitree_lidar. Without this relay,
#      local_costmap starves, controller_server aborts every goal, and
#      frontier_explorer blacklists every pose. The relay bridges the gap.
#   3. Publishes goal on /goal_pose
#
# Prerequisites:
#   01_start_demo.sh and 05_start_nav.sh must have run first.
#   It helps to have driven Go2 around a bit with 02_walk.sh first so SLAM
#   has built some map.
#
# Usage:
#   ./06_send_goal.sh                    # default: x=2, y=0, yaw=0
#   ./06_send_goal.sh 5 0                # go to (5, 0)
#   ./06_send_goal.sh 3 -2 1.57          # go to (3, -2) facing 90°
#   ./06_send_goal.sh cancel             # cancel current goal
#   ./06_send_goal.sh status             # full diagnostic of nav stack

set -e

# ============================================================
# Special commands first
# ============================================================
ACTION="${1:-default}"

case "$ACTION" in
  cancel)
    echo "Cancelling current navigation goal..."
    docker exec cmd_vel_publisher bash -c '
      source /opt/ros/jazzy/setup.bash
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=42
      timeout 2 ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
        "{header: {frame_id: \"map\"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" >/dev/null 2>&1 || true
    ' 2>/dev/null
    echo "Sent stop pose (origin). Go2 should halt within a few seconds."
    exit 0
    ;;

  status)
    echo "==============================================="
    echo " Panthera nav stack — status"
    echo "==============================================="
    echo ""
    echo "[Containers]"
    for c in panthera_om1_demo cmd_vel_publisher panthera_nav; do
      state=$(docker ps --filter "name=^${c}$" --format '{{.Status}}' 2>/dev/null)
      [ -z "$state" ] && state="NOT RUNNING"
      printf "  %-22s : %s\n" "$c" "$state"
    done
    echo ""
    echo "[Topic flow rates]"
    docker exec cmd_vel_publisher bash -c '
      source /opt/ros/jazzy/setup.bash
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=42
      for t in /scan /odom /unitree_lidar /utlidar/cloud_deskewed /map /plan /cmd_vel_nav /cmd_vel; do
        rate=$(timeout 3 ros2 topic hz "$t" 2>&1 | grep "average rate" | head -1 | sed "s/average rate: //")
        [ -z "$rate" ] && rate="(silent)"
        printf "  %-30s : %s\n" "$t" "$rate"
      done
    ' 2>&1 | grep -vE "^\[WARN\]|^WARNING:"
    echo ""
    echo "[Relay process in cmd_vel_publisher]"
    docker exec cmd_vel_publisher bash -c 'ps aux | grep -i topic_tools | grep -v grep' || echo "  (no relay running)"
    echo ""
    echo "[Nav2 lifecycle states]"
    docker exec panthera_nav bash -c '
      source /opt/ros/humble/setup.bash
      source /app/om1_ros2_sdk/install/setup.bash
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=42
      for node in controller_server planner_server bt_navigator behavior_server smoother_server velocity_smoother; do
        state=$(timeout 3 ros2 lifecycle get /$node 2>&1 | head -1)
        printf "  %-22s : %s\n" "/$node" "$state"
      done
    ' 2>&1 | grep -vE "^\[WARN\]|^WARNING:"
    exit 0
    ;;

  help|--help|-h)
    echo "Usage: $0 [x] [y] [yaw_radians]"
    echo "       $0 cancel    # stop by sending an origin goal"
    echo "       $0 status    # full diagnostic of nav stack"
    echo ""
    echo "Examples:"
    echo "  $0           # default: 2 m forward (x=2, y=0)"
    echo "  $0 5 0       # go to (5, 0)"
    echo "  $0 3 -2      # go to (3, -2)"
    echo "  $0 0 0 1.57  # return to origin facing 90°"
    exit 0
    ;;
esac

# ============================================================
# Goal mode
# ============================================================
GOAL_X="${1:-2.0}"
GOAL_Y="${2:-0.0}"
GOAL_YAW="${3:-0.0}"

# Validate numeric input
if ! [[ "$GOAL_X" =~ ^-?[0-9]+\.?[0-9]*$ ]]; then
  echo "ERROR: '$GOAL_X' is not a valid number. Use '$0 help'."
  exit 1
fi

# Convert yaw → quaternion
QZ=$(python3 -c "import math; print(math.sin($GOAL_YAW * 0.5))")
QW=$(python3 -c "import math; print(math.cos($GOAL_YAW * 0.5))")

echo "==============================================="
echo " Panthera demo — autonomous nav goal"
echo "==============================================="
echo ""
echo " Target: x=$GOAL_X, y=$GOAL_Y, yaw=$GOAL_YAW rad"
echo ""

# ============================================================
# Pre-flight: containers
# ============================================================
echo "[1/5] Container check..."
for c in panthera_om1_demo cmd_vel_publisher panthera_nav; do
  if ! docker ps --format '{{.Names}}' | grep -q "^${c}$"; then
    echo "  ✗ $c not running"
    case "$c" in
      panthera_om1_demo|cmd_vel_publisher) echo "    Run ./01_start_demo.sh first." ;;
      panthera_nav) echo "    Run ./05_start_nav.sh first." ;;
    esac
    exit 1
  fi
done
echo "  ✓ all 3 containers up"

# ============================================================
# Pre-flight: ros-jazzy-topic-tools must be installed
# ============================================================
echo ""
echo "[2/5] Verifying topic_tools is installed in cmd_vel_publisher..."
HAS_TT=$(docker exec cmd_vel_publisher bash -c '
  source /opt/ros/jazzy/setup.bash
  ros2 pkg list 2>/dev/null | grep -x topic_tools | head -1
')

if [ -z "$HAS_TT" ]; then
  echo "  topic_tools missing — installing ros-jazzy-topic-tools..."
  docker exec cmd_vel_publisher bash -c '
    apt-get update -qq >/dev/null 2>&1
    apt-get install -y ros-jazzy-topic-tools >/dev/null 2>&1
  '
  HAS_TT_AFTER=$(docker exec cmd_vel_publisher bash -c '
    source /opt/ros/jazzy/setup.bash
    ros2 pkg list 2>/dev/null | grep -x topic_tools | head -1
  ')
  if [ -z "$HAS_TT_AFTER" ]; then
    echo "  ✗ Install failed. Try manually:"
    echo "    docker exec cmd_vel_publisher apt install -y ros-jazzy-topic-tools"
    exit 1
  fi
  echo "  ✓ topic_tools installed"
else
  echo "  ✓ topic_tools already installed"
fi

# ============================================================
# Pre-flight: relay /unitree_lidar → /utlidar/cloud_deskewed
# ============================================================
echo ""
echo "[3/5] Verifying lidar relay is running..."

RELAY_PID=$(docker exec cmd_vel_publisher bash -c \
  'ps -ef | grep "topic_tools relay /unitree_lidar /utlidar/cloud_deskewed" | grep -v grep | awk "{print \$2}" | head -1')

if [ -z "$RELAY_PID" ]; then
  echo "  Relay not running — starting it..."
  docker exec -d cmd_vel_publisher bash -c '
    source /opt/ros/jazzy/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID=42
    exec ros2 run topic_tools relay /unitree_lidar /utlidar/cloud_deskewed > /tmp/relay.log 2>&1
  '
  sleep 3

  RELAY_PID_AFTER=$(docker exec cmd_vel_publisher bash -c \
    'ps -ef | grep "topic_tools relay /unitree_lidar /utlidar/cloud_deskewed" | grep -v grep | awk "{print \$2}" | head -1')

  if [ -z "$RELAY_PID_AFTER" ]; then
    echo "  ✗ Relay failed to start. Log:"
    docker exec cmd_vel_publisher cat /tmp/relay.log 2>&1 | head -10
    exit 1
  fi
  echo "  ✓ Relay started (PID $RELAY_PID_AFTER)"
  echo "  Waiting 12 sec for local_costmap to recover from stale buffer..."
  sleep 12
else
  echo "  ✓ Relay already running (PID $RELAY_PID)"
fi

# Verify the relay is actually publishing
RELAY_RATE=$(docker exec cmd_vel_publisher bash -c '
  source /opt/ros/jazzy/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=42
  timeout 3 ros2 topic hz /utlidar/cloud_deskewed 2>&1 | grep "average rate" | head -1
')
if [ -z "$RELAY_RATE" ]; then
  echo "  ⚠ /utlidar/cloud_deskewed shows no rate. Local costmap may stay starved."
else
  echo "  ✓ /utlidar/cloud_deskewed flowing: $RELAY_RATE"
fi

# ============================================================
# Pre-flight: /map publisher
# ============================================================
echo ""
echo "[4/5] Checking /map availability..."
HAS_MAP=$(docker exec cmd_vel_publisher bash -c '
  source /opt/ros/jazzy/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=42
  timeout 3 ros2 topic info /map 2>&1 | grep -i "Publisher count" | head -1
')
echo "  $HAS_MAP"
if echo "$HAS_MAP" | grep -q "Publisher count: 0"; then
  echo "  ⚠ No /map publisher. SLAM may not be ready."
  echo "  Tip: drive around with ./02_walk.sh circle first to build a map."
fi

# ============================================================
# Send the goal
# ============================================================
echo ""
echo "[5/5] Publishing goal pose..."
docker exec cmd_vel_publisher bash -c "
  source /opt/ros/jazzy/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID=42
  ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
    '{header: {frame_id: \"map\"}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}' 2>&1 | grep -vE 'WARN|WARNING' | tail -5
"

echo ""
echo "==============================================="
echo " Goal sent."
echo "==============================================="
echo ""
echo " 👀 Look at VNC. You should see:"
echo "   - Isaac Sim: Go2 walking toward the goal"
echo "   - RViz (if open):"
echo "       * Path visualization (the planned route)"
echo "       * Local + global costmaps"
echo "       * Robot moving along the path"
echo ""
echo " Verify cmd_vel is flowing:"
echo "   ./06_send_goal.sh status"
echo ""
echo " To stop:"
echo "   ./06_send_goal.sh cancel"
echo ""
