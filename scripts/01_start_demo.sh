#!/bin/bash
# Panthera demo — cold start
# Brings up Isaac Sim + Go2 + walking policy, ready for cmd_vel.
# Run this FIRST. Then run 02_walk.sh once the sim window appears in VNC.

set -e

echo "==============================================="
echo " Panthera demo — cold start"
echo "==============================================="
echo ""
echo "[1/5] Stopping any existing containers..."
docker rm -f panthera_om1_demo cmd_vel_publisher 2>/dev/null || true

echo ""
echo "[2/5] Starting Isaac Sim container (panthera_om1_demo)..."
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
  -v $HOME/panthera/om1_demo/cache/kit:/isaac-sim/kit/cache:rw \
  -v $HOME/panthera/om1_demo/cache/ov:/isaac-sim/.cache/ov:rw \
  -v $HOME/panthera/om1_demo/cache/pip:/isaac-sim/.cache/pip:rw \
  -v $HOME/panthera/om1_demo/cache/gl:/isaac-sim/.cache/nvidia/GLCache:rw \
  -v $HOME/panthera/om1_demo/cache/compute:/isaac-sim/.nv/ComputeCache:rw \
  -v $HOME/panthera/om1_demo/cache/warp:/isaac-sim/.cache/warp:rw \
  -v $HOME/panthera/om1_demo/isaac_sim:/workspace/om1_isaac:rw \
  --entrypoint bash \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  -c "tail -f /dev/null" > /dev/null
sleep 3
echo "    Isaac Sim container up."

echo ""
echo "[3/5] Starting cmd_vel publisher container..."
docker run -d --name cmd_vel_publisher \
  --network host \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID=42 \
  ros:jazzy-ros-base \
  sleep infinity > /dev/null
sleep 2

# Install Cyclone in publisher (skipped if already cached)
echo "    Ensuring Cyclone RMW installed in publisher..."
docker exec cmd_vel_publisher bash -c '
apt update -qq 2>&1 | tail -1
apt install -y -qq ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-geometry-msgs 2>&1 | tail -1
' > /dev/null
echo "    Publisher container up."

echo ""
echo "[4/5] Launching run.py inside Isaac Sim (this takes ~60 sec)..."
docker exec -d panthera_om1_demo bash -c '
cd /workspace/om1_isaac && /isaac-sim/python.sh run.py --robot_type go2 --no_keyboard --no_sensors > /tmp/demo.log 2>&1
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
echo "[5/5] Verification:"
docker exec panthera_om1_demo bash -c 'grep "PANTHERA" /tmp/demo.log | tail -5'

echo ""
echo "==============================================="
echo " Cold start complete."
echo "==============================================="
echo ""
echo " 1. Open VNC: http://92.180.27.82:51954 (or your VNC port)"
echo " 2. You should see Isaac Sim with Go2 standing on a warehouse floor."
echo "    (Tip: top-right of viewport, switch 'Top' → 'Perspective')"
echo "    (Tip: search 'Go2' in Stage panel, click it, press F to frame)"
echo ""
echo " 3. To make Go2 WALK, run:  ~/panthera/om1_demo/scripts/02_walk.sh forward"
echo " 4. To STOP everything:     ~/panthera/om1_demo/scripts/03_stop_demo.sh"
echo ""
