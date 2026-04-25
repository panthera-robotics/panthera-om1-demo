#!/bin/bash
# Panthera: Launch Go2 in Isaac Sim 5.1 with visible window
# Run this FROM the Selkies desktop terminal so Isaac Sim displays in browser

set -e

echo "=== Patch run.py to non-headless ==="
sed -i 's/SimulationApp({"renderer": "RaytracedLighting", "headless": True})/SimulationApp({"renderer": "RaytracedLighting", "headless": False})/' \
  /root/panthera/om1_demo/isaac_sim/run.py
grep "SimulationApp(" /root/panthera/om1_demo/isaac_sim/run.py | head -1

echo ""
echo "=== DISPLAY check ==="
echo "Current DISPLAY: $DISPLAY"
if [ -z "$DISPLAY" ]; then
  echo "DISPLAY is empty — trying :0"
  export DISPLAY=:0
fi
echo "Using DISPLAY: $DISPLAY"

echo ""
echo "=== Allow container to access this X server ==="
xhost +local:docker 2>&1 | head -1 || echo "(xhost may need install)"

echo ""
echo "=== Recreate container with X11 mounts ==="
docker rm -f panthera_om1_demo 2>/dev/null

docker run -d --name panthera_om1_demo --runtime=nvidia --gpus all \
  --user 1234:1234 \
  --network host \
  --ipc host \
  -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" -e "OMNI_KIT_ALLOW_ROOT=1" \
  -e "HOME=/isaac-sim" \
  -e "ROS_DISTRO=jazzy" \
  -e "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
  -e "ROS_DOMAIN_ID=42" \
  -e "LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib" \
  -e "DISPLAY=$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/panthera/om1_demo/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/panthera/om1_demo/cache/ov:/isaac-sim/.cache/ov:rw \
  -v ~/panthera/om1_demo/cache/pip:/isaac-sim/.cache/pip:rw \
  -v ~/panthera/om1_demo/cache/gl:/isaac-sim/.cache/nvidia/GLCache:rw \
  -v ~/panthera/om1_demo/cache/compute:/isaac-sim/.nv/ComputeCache:rw \
  -v ~/panthera/om1_demo/isaac_sim:/workspace/om1_isaac:rw \
  --entrypoint bash \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  -c "tail -f /dev/null"

sleep 3
docker ps | grep panthera_om1_demo

echo ""
echo "=== Launch Isaac Sim with Go2 ==="
echo "Window should appear on this desktop in ~30-60 sec"
echo "Press Ctrl+C in this terminal to stop"
echo ""

docker exec panthera_om1_demo bash -c "
cd /workspace/om1_isaac && /isaac-sim/python.sh run.py --robot_type go2 --no_keyboard --no_sensors 2>&1
"
