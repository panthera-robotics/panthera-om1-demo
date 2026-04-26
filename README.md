# Panthera OM1 Demo on Vast.ai (WIP — debug snapshot)

OM1's Isaac Sim 5.1 launcher + walking policies, plus diagnostic patches.

## Status snapshot (April 25, 2026 — late night)

### Confirmed working
- Isaac Sim 5.1.0 NGC image runs on Vast RTX 5090
- Cache permissions work when container runs as UID 1234 (isaac-sim user)
- ROS2 Jazzy bridge loads inside container (`rclpy loaded`)
- CycloneDDS proven cross-container on Vast with `network_mode: host`
  (separate validation test, not Isaac Sim integrated)
- OM1's RobotRosRunner constructs successfully
- Go2 USD spawns in Isaac Sim scene (`/World/Go2/base/...`)
- Walking policies extracted from openmindagi/om1_ros2_sdk:v1.0.1

### Current blocker
`runner.setup_ros()` dies between markers E2 and E3. Specifically:

  [PANTHERA-MARK] E2: self._cmd_vel_topic = /cmd_vel  ← prints OK
  <next line dies, no traceback>
  [Simulation App Shutting Down]

The next line is:
  self._linear_attr, ... = ros_utils.setup_cmd_vel_graph(self._cmd_vel_topic)

We patched setup_cmd_vel_graph to use rclpy directly (bypassing OmniGraph)
but it still dies. Function body's first print() never fires.
Suggests the call itself crashes Isaac Sim's interpreter before
function entry.

### Container config that gets us this far

  image: nvcr.io/nvidia/isaac-sim:5.1.0
  user: 1234:1234
  network_mode: host
  HOME: /isaac-sim
  ROS_DISTRO: jazzy
  RMW_IMPLEMENTATION: rmw_fastrtps_cpp
  ROS_DOMAIN_ID: 42
  LD_LIBRARY_PATH: /isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib
  Cache mounts at /isaac-sim/.cache/* (NOT /root/.cache — user is isaac-sim)

### Tomorrow morning's first step

Try OM1's documented setup path: pip-install Isaac Sim 5.1 in a Python 3.11
venv on the Vast host (per OM1 docs), use Humble DDS instead of Jazzy.
That's the OM1-tested combination. If walking works there, then the
bug is specifically in NGC-image + Jazzy combination.

If still failing: ask in OM1 Discord with this README as context.

## MILESTONE — April 25, 2026 ~23:38 UTC

**Go2 visible and walking-policy-active in Isaac Sim 5.1 on Vast.ai**, streamed
to laptop browser via Selkies desktop on port 6100 → 51683.

Configuration that worked:
- Container: nvcr.io/nvidia/isaac-sim:5.1.0 as user 1234 (isaac-sim)
- DISPLAY=:0 from host Selkies session, X11 socket bind-mounted
- run.py with `headless: False` and setup_ros() bypassed
- Walking policy from openmindagi/om1_ros2_sdk:v1.0.1

Confirmed via PANTHERA-MARK markers reaching G ("before runner.run() — entering main loop")
and visual confirmation of Go2 quadruped in scene viewport.

Next: re-enable setup_ros (or its rclpy bypass) so external cmd_vel topics drive Go2.

## MILESTONE — April 26, 2026

**Full ROS2 control loop working: external cmd_vel publisher → rclpy subscriber inside
Isaac Sim 5.1 → walking policy → Go2 motion.**

### Issue 14 (from Problems Faced doc) — RESOLVED

Three compounding causes were discovered in this order:

1. **Warp cache permission denied.** `/isaac-sim/.cache/warp` not bind-mounted →
   omni.replicator.core extension failed silently → cascade of swallowed exceptions.
   **Fix:** add `~/panthera/om1_demo/cache/warp:/isaac-sim/.cache/warp:rw` mount,
   chowned to 1234:1234.

2. **`utils.py` name collision with OpenCV.** Isaac Sim's pip prebundle includes
   `cv2/utils/__init__.py` which gets registered in `sys.modules` as `utils` before
   our path is added. `import utils as ros_utils` in run.py was loading OpenCV's
   utils package, not our patched file. Calls to `ros_utils.setup_cmd_vel_graph(...)`
   raised AttributeError silently inside Isaac Sim's call machinery.
   **Fix:** rename `utils.py` → `om1_utils.py`, update `run.py` import line to
   `import om1_utils as ros_utils`.

3. **rclpy not directly importable.** Isaac Sim's bundled rclpy is loaded as part
   of `isaacsim.ros2.bridge` extension startup. `run.py` already enables this
   extension before calling `setup_ros()`, so the original code path works once
   the above two bugs are fixed.

### Verification chain

- `[T1] returned: tuple of length 3 — _AttrLike, _AttrLike, _CountAttr` (isolated test)
- `[PANTHERA] cmd_vel subscriber active on /cmd_vel (rclpy bypass)` (full run.py)
- `[PANTHERA-MARK] G: before runner.run() — entering main loop` (sim entered policy loop)
- Exit code 124 on 200-sec timeout (sim still running when killed)
- External `cmd_vel_publisher` container publishes Twist at 10 Hz from
  `ros:jazzy-ros-base` with default Cyclone DDS, network_mode: host
- `/cmd_vel` topic visible from publisher's `ros2 topic list`
- Topic hz confirmed: `average rate: 10.004 Hz, std dev 0.00018s`

### Architecture validated

External ROS2 publisher (separate container) → CycloneDDS over network_mode: host →
rclpy subscriber inside Isaac Sim 5.1 → linear_attr.set([0.4, 0, 0]) →
walking policy reads .get() at every physics step → Go2 walks forward.

This is the production-ready architecture. Same containers, same DDS configuration
will run on real Jetson + Go2 hardware in May. No XML peer files, no TCP unicast,
no provider-specific glue.

