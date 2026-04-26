#!/bin/bash
# Panthera demo — view sensor data in RViz
# Opens RViz in the VNC desktop, pre-configured to show:
#   - 2D LiDAR (/scan)            → red laser dots
#   - 3D LiDAR (/unitree_lidar)   → colored pointcloud
#   - Front camera (/camera/go2/image_raw)
#   - RealSense color (/camera/realsense2_camera_node/color/...)
#   - Odometry (/odom)            → arrow showing robot pose
#   - TF frames                   → robot link transforms
#
# Prerequisites:
#   01_start_demo.sh must already be running (Isaac Sim publishing topics)
#   Publisher container must have been created with X11 mounts (true if
#   01_start_demo.sh was run after the X11 update).
#
# Usage:
#   ./04_view_sensors.sh

set -e

echo "==============================================="
echo " Panthera demo — RViz sensor viewer"
echo "==============================================="
echo ""

# Sanity check: containers exist
if ! docker ps --format '{{.Names}}' | grep -q '^panthera_om1_demo$'; then
    echo "ERROR: panthera_om1_demo container is not running."
    echo "       Run ./01_start_demo.sh first."
    exit 1
fi
if ! docker ps --format '{{.Names}}' | grep -q '^cmd_vel_publisher$'; then
    echo "ERROR: cmd_vel_publisher container is not running."
    echo "       Run ./01_start_demo.sh first."
    exit 1
fi

# Sanity check: publisher has X11 mount (will fail to launch RViz otherwise)
if ! docker inspect cmd_vel_publisher --format '{{range .Mounts}}{{.Destination}} {{end}}' | grep -q "/tmp/.X11-unix"; then
    echo "ERROR: cmd_vel_publisher was created without X11 forwarding."
    echo "       This usually means 01_start_demo.sh predates the X11-aware version."
    echo "       Stop everything and rerun:"
    echo "         ./03_stop_demo.sh"
    echo "         ./01_start_demo.sh"
    exit 1
fi

# Re-grant X access (cheap, idempotent)
DESKTOP_USER="${DESKTOP_USER:-user}"
XAUTH_FILE="/home/${DESKTOP_USER}/.Xauthority"
sudo -u "$DESKTOP_USER" DISPLAY=:0 XAUTHORITY="$XAUTH_FILE" xhost +local: > /dev/null 2>&1 || true

# Ensure RViz is installed in publisher (idempotent)
echo "[1/3] Ensuring RViz is installed in publisher container..."
docker exec cmd_vel_publisher bash -c '
which rviz2 > /dev/null 2>&1 && echo "    rviz2 already installed" || {
  apt update -qq 2>&1 | tail -1
  apt install -y -qq ros-jazzy-rviz2 2>&1 | tail -1
}
'

echo ""
echo "[2/3] Writing RViz config (all sensors)..."
docker exec cmd_vel_publisher bash -c 'cat > /tmp/panthera.rviz <<RVIZ
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Grid
    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
    - Class: rviz_default_plugins/LaserScan
      Enabled: true
      Name: LaserScan (2D)
      Topic:
        Value: /scan
      Size (m): 0.05
      Color: 255; 0; 0
    - Class: rviz_default_plugins/PointCloud2
      Enabled: true
      Name: 3D LiDAR
      Topic:
        Value: /unitree_lidar
      Size (m): 0.03
      Color Transformer: AxisColor
    - Class: rviz_default_plugins/Image
      Enabled: true
      Name: Front Camera
      Topic:
        Value: /camera/go2/image_raw
    - Class: rviz_default_plugins/Image
      Enabled: false
      Name: RealSense Color
      Topic:
        Value: /camera/realsense2_camera_node/color/image_isaac_sim_raw
    - Class: rviz_default_plugins/Odometry
      Enabled: true
      Name: Odometry
      Topic:
        Value: /odom
  Global Options:
    Fixed Frame: odom
    Background Color: 48; 48; 48
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
RVIZ
'

echo ""
echo "[3/3] Launching RViz (appears in VNC desktop)..."

# Kill any previous RViz to avoid duplicate windows
docker exec cmd_vel_publisher pkill -f rviz2 2>/dev/null || true
sleep 1

docker exec -d cmd_vel_publisher bash -c '
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export DISPLAY=:0
export XAUTHORITY=/tmp/.docker.xauth
rviz2 -d /tmp/panthera.rviz > /tmp/rviz.log 2>&1
'

echo "    Waiting 6 sec for RViz to render..."
sleep 6

if docker exec cmd_vel_publisher pgrep -af rviz2 > /dev/null 2>&1; then
    echo "    RViz process running."
else
    echo "    WARNING: rviz2 process not found. Check logs:"
    echo "      docker exec cmd_vel_publisher tail -20 /tmp/rviz.log"
fi

echo ""
echo "==============================================="
echo " RViz launched."
echo "==============================================="
echo ""
echo " 👀 Switch to VNC. Two windows should be visible:"
echo "    - Isaac Sim (Go2 in the world)"
echo "    - RViz (sensor data: LiDAR, cameras, TF, odometry)"
echo ""
echo " Tip: drive Go2 around with ./02_walk.sh and watch the LiDAR"
echo "      scan + odometry arrow update in RViz in real time."
echo ""
echo " To close RViz only:  docker exec cmd_vel_publisher pkill -f rviz2"
echo ""
