#!/bin/bash
# Panthera demo — bootstrap a fresh environment
# Run this ONCE on a fresh Vast.ai instance (or any fresh Docker host)
# to set up everything required to run the demo. Idempotent — safe to
# re-run if something failed partway through.
#
# What this does:
#   1. Pulls Docker images (Isaac Sim 5.1, ROS Jazzy, OM1 SDK)
#   2. Creates cache directories with correct UID 1234 ownership
#   3. Extracts walking policy + Go2 USD assets from OM1 image
#   4. Verifies the patched run.py and om1_utils.py from git survived
#
# After this, run ./scripts/01_start_demo.sh to bring up the demo.

set -e

REPO_DIR="$HOME/panthera/panthera-om1-demo"
CACHE_DIR="$REPO_DIR/cache"
ISAAC_SIM_DIR="$REPO_DIR/isaac_sim"

echo "==============================================="
echo " Panthera demo — bootstrap (fresh environment)"
echo "==============================================="
echo ""

# Sanity: are we in the right place?
if [ ! -d "$REPO_DIR" ]; then
    echo "ERROR: $REPO_DIR does not exist."
    echo "       Did you clone the repo? Expected location:"
    echo "       ~/panthera/panthera-om1-demo (with hyphens)"
    exit 1
fi

if [ ! -d "$ISAAC_SIM_DIR" ]; then
    echo "ERROR: $ISAAC_SIM_DIR does not exist."
    echo "       The repo seems incomplete. Re-clone from GitHub."
    exit 1
fi

echo "[1/5] Pulling Docker images in parallel..."
echo "      - nvcr.io/nvidia/isaac-sim:5.1.0   (~22 GB)"
echo "      - ros:jazzy-ros-base               (~600 MB)"
echo "      - openmindagi/om1_ros2_sdk:v1.0.1  (~9 GB)"
echo ""

docker pull nvcr.io/nvidia/isaac-sim:5.1.0 > /tmp/pull_isaac.log 2>&1 &
PID_ISAAC=$!
docker pull ros:jazzy-ros-base > /tmp/pull_ros.log 2>&1 &
PID_ROS=$!
docker pull openmindagi/om1_ros2_sdk:v1.0.1 > /tmp/pull_om1.log 2>&1 &
PID_OM1=$!

echo "      Waiting (this can take 5-15 min depending on bandwidth)..."
while kill -0 $PID_ISAAC 2>/dev/null || kill -0 $PID_ROS 2>/dev/null || kill -0 $PID_OM1 2>/dev/null; do
    sleep 30
    echo "      ... still pulling at $(date +%H:%M:%S)"
done

# Verify pulls succeeded
for label in isaac ros om1; do
    pid_var="PID_${label^^}"
    pid="${!pid_var}"
    wait $pid
    rc=$?
    if [ $rc -ne 0 ]; then
        echo "      ERROR: $label pull failed (exit $rc). See /tmp/pull_${label}.log"
        exit 1
    fi
done
echo "      All images pulled."

echo ""
echo "[2/5] Creating cache directories with UID 1234 ownership..."
mkdir -p "$CACHE_DIR"/{kit,ov,pip,gl,compute,warp}
chown -R 1234:1234 "$CACHE_DIR"
ls -la "$CACHE_DIR" | head -10
echo "      Caches ready."

echo ""
echo "[3/5] Extracting walking policy and assets from OM1 SDK image..."

# Clean up any leftover container from a previous failed run
docker rm -f om1_extract > /dev/null 2>&1 || true

docker create --name om1_extract openmindagi/om1_ros2_sdk:v1.0.1 > /dev/null
mkdir -p /tmp/om1_isaac_extract
docker cp om1_extract:/app/om1_ros2_sdk/unitree/isaac_sim/. /tmp/om1_isaac_extract/
docker rm om1_extract > /dev/null

echo "      Extracted dirs:"
ls /tmp/om1_isaac_extract/ | sed 's/^/        /'

echo ""
echo "      Copying into $ISAAC_SIM_DIR (cp -n: never overwrite git-tracked files)..."
cp -rn /tmp/om1_isaac_extract/* "$ISAAC_SIM_DIR/" 2>&1 || true
rm -rf /tmp/om1_isaac_extract

echo ""
echo "[4/5] Verifying critical files exist..."
MISSING=0

check_file() {
    if [ -e "$1" ]; then
        echo "      ✓ $(basename $1) ($(du -h "$1" | cut -f1))"
    else
        echo "      ✗ MISSING: $1"
        MISSING=$((MISSING+1))
    fi
}

check_file "$ISAAC_SIM_DIR/run.py"
check_file "$ISAAC_SIM_DIR/om1_utils.py"
check_file "$ISAAC_SIM_DIR/checkpoints/go2/exported/policy.pt"
check_file "$ISAAC_SIM_DIR/checkpoints/go2/params/env.yaml"
check_file "$ISAAC_SIM_DIR/checkpoints/g1/exported/policy.pt"
check_file "$ISAAC_SIM_DIR/checkpoints/g1/params/env.yaml"

if [ $MISSING -gt 0 ]; then
    echo ""
    echo "      ERROR: $MISSING required file(s) missing. Bootstrap incomplete."
    exit 1
fi

echo ""
echo "[5/5] Verifying Panthera patches are intact in run.py and om1_utils.py..."

if grep -q "PANTHERA-MARK" "$ISAAC_SIM_DIR/run.py"; then
    echo "      ✓ run.py has PANTHERA-MARK debug tracers"
else
    echo "      ✗ run.py is missing PANTHERA-MARK — was it overwritten?"
    echo "        Re-checkout from git: cd $REPO_DIR && git checkout isaac_sim/run.py"
    exit 1
fi

if grep -q "PANTHERA PATCH" "$ISAAC_SIM_DIR/om1_utils.py"; then
    echo "      ✓ om1_utils.py has PANTHERA PATCH (rclpy bypass)"
else
    echo "      ✗ om1_utils.py is missing PANTHERA PATCH — was it overwritten?"
    echo "        Re-checkout from git: cd $REPO_DIR && git checkout isaac_sim/om1_utils.py"
    exit 1
fi

echo ""
echo "==============================================="
echo " Bootstrap complete."
echo "==============================================="
echo ""
echo " Next step:    ./scripts/01_start_demo.sh"
echo ""
echo " Then in browser:"
echo "   - Selkies (HTTP):   http://<vast-ip>:<VAST_TCP_PORT_6100>"
echo "   - VNC (raw):        <vast-ip>:<VAST_TCP_PORT_5900>"
echo ""
