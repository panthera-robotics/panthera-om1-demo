#!/bin/bash
# Panthera demo — cold start with X11 forwarding to VNC desktop
# Brings up Isaac Sim + Go2 + walking policy, with the sim window
# rendering on the host's X display so it appears in VNC/Selkies.
#
# Run this FIRST. Then run 02_walk.sh once you see Go2 in VNC.

set -e

# Source robot config (sets PANTHERA_ROBOT_TYPE based on ROBOT env var)
source "$(dirname "$0")/lib/robot_config.sh"

echo "==============================================="
echo " Panthera demo — cold start (ROBOT=$ROBOT)"
echo "==============================================="
echo ""

# Detect host X display config
HOST_DISPLAY="${DISPLAY:-:0}"
DESKTOP_USER="${DESKTOP_USER:-user}"
XAUTH_FILE="/home/${DESKTOP_USER}/.Xauthority"

echo "[1/6] X11 forwarding setup"
echo "    Host DISPLAY: $HOST_DISPLAY"
echo "    Desktop user: $DESKTOP_USER"
echo "    Xauthority:   $XAUTH_FILE"

if [ ! -e /tmp/.X11-unix/X0 ]; then
    echo "    WARNING: /tmp/.X11-unix/X0 does not exist."
    echo "    The desktop X server may not be running."
    echo "    Continuing anyway — Isaac Sim will fall back to headless."
fi

# Allow local Docker connections to X server (run as desktop user since they own the X session)
if [ -f "$XAUTH_FILE" ]; then
    sudo -u "$DESKTOP_USER" DISPLAY="$HOST_DISPLAY" XAUTHORITY="$XAUTH_FILE" xhost +local: > /dev/null 2>&1 \
        && echo "    xhost +local: granted" \
        || echo "    xhost failed (you may need to run 'xhost +local:' manually in the VNC terminal)"
else
    echo "    No Xauthority file at $XAUTH_FILE — skipping xhost"
fi

echo ""
echo "[2/6] Stopping any existing containers..."
docker rm -f panthera_om1_demo cmd_vel_publisher 2>/dev/null > /dev/null || true

echo ""
echo "[3/6] Starting Isaac Sim container (panthera_om1_demo) with X11 forwarding..."
docker run -d --name panthera_om1_demo --runtime=nvidia --gpus all \
  --user 1234:1234 \
  --network host \
  --ipc host \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e OMNI_KIT_ALLOW_ROOT=1 \
  -e HOME=/isaac-sim \
  -e ROS_DISTRO=jazzy \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID=42 \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib \
  -e DISPLAY="$HOST_DISPLAY" \
  -e XAUTHORITY=/tmp/.docker.xauth \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$XAUTH_FILE":/tmp/.docker.xauth:ro \
  -v "$HOME/panthera/panthera-om1-demo/cache/kit":/isaac-sim/kit/cache:rw \
  -v "$HOME/panthera/panthera-om1-demo/cache/ov":/isaac-sim/.cache/ov:rw \
  -v "$HOME/panthera/panthera-om1-demo/cache/pip":/isaac-sim/.cache/pip:rw \
  -v "$HOME/panthera/panthera-om1-demo/cache/gl":/isaac-sim/.cache/nvidia/GLCache:rw \
  -v "$HOME/panthera/panthera-om1-demo/cache/compute":/isaac-sim/.nv/ComputeCache:rw \
  -v "$HOME/panthera/panthera-om1-demo/cache/warp":/isaac-sim/.cache/warp:rw \
  -v "$HOME/panthera/panthera-om1-demo/isaac_sim":/workspace/om1_isaac:rw \
  --entrypoint bash \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  -c "tail -f /dev/null" > /dev/null
sleep 3
echo "    Isaac Sim container up."

echo ""
echo "[4/6] Starting cmd_vel publisher container..."
docker run -d --name cmd_vel_publisher \
  --network host \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID=42 \
  -e DISPLAY="$HOST_DISPLAY" \
  -e XAUTHORITY=/tmp/.docker.xauth \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$XAUTH_FILE":/tmp/.docker.xauth:ro \
  ros:jazzy-ros-base \
  sleep infinity > /dev/null
sleep 2

echo "    Ensuring Cyclone RMW + geometry_msgs installed..."
docker exec cmd_vel_publisher bash -c '
apt update -qq 2>&1 | tail -1
apt install -y -qq ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-geometry-msgs ros-jazzy-rviz2 2>&1 | tail -1
' > /dev/null
echo "    Publisher container up."

echo ""
echo "[5/6] Launching run.py inside Isaac Sim (this takes ~60 sec)..."
docker exec -d panthera_om1_demo bash -c '
cd /workspace/om1_isaac && /isaac-sim/python.sh run.py --robot_type $PANTHERA_ROBOT_TYPE --no_keyboard  > /tmp/demo.log 2>&1
'

echo "    Waiting for sim to enter main loop..."
for i in {1..18}; do
  sleep 5
  if docker exec panthera_om1_demo bash -c 'grep -q "before runner.run()" /tmp/demo.log 2>/dev/null'; then
    echo "    Main loop reached at $((i*5)) sec."
    break
  fi
  echo "    ...still booting ($((i*5)) sec)"
done

echo ""
echo "[6/6] Verification:"
docker exec panthera_om1_demo bash -c 'grep "PANTHERA" /tmp/demo.log | tail -5'

echo ""
echo "==============================================="
echo " Cold start complete."
echo "==============================================="
echo ""
echo " 1. Open Selkies in browser:  http://92.180.27.82:51683"
echo "    (or VNC client to:        92.180.27.82:51954)"
echo ""
echo " 2. Isaac Sim window with Go2 should be visible on the desktop."
echo "    Tip: switch viewport from 'Top' to 'Perspective' (top-right)"
echo "    Tip: search 'Go2' in Stage panel, click it, press F to frame"
echo ""
echo " 3. Make Go2 walk:    ~/panthera/panthera-om1-demo/scripts/02_walk.sh forward"
echo "    Other commands:    back | turn | circle | stop"
echo ""
echo " 4. When done:        ~/panthera/panthera-om1-demo/scripts/03_stop_demo.sh"
echo ""
