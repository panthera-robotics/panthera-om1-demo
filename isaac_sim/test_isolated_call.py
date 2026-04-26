"""Isolated test — enables ros2 bridge so rclpy is available."""
from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": True})
print("[T1] SimApp up", flush=True)

from isaacsim.core.utils import extensions
extensions.enable_extension("isaacsim.ros2.bridge")
sim_app.update()
print("[T1] ros2 bridge extension enabled", flush=True)

import sys
sys.path.insert(0, "/workspace/om1_isaac")
import om1_utils as utils
print("[T1] om1_utils imported", flush=True)

try:
    print("[T1] about to call setup_cmd_vel_graph", flush=True)
    result = utils.setup_cmd_vel_graph("/test_topic")
    print(f"[T1] returned: tuple of length {len(result)}", flush=True)
    print(f"[T1] result types: {[type(r).__name__ for r in result]}", flush=True)
    # Check the API contract: each result needs a .get() method
    for i, r in enumerate(result):
        has_get = hasattr(r, "get")
        print(f"[T1]   result[{i}] hasattr(.get): {has_get}", flush=True)
except BaseException as e:
    import traceback
    print(f"[T1] EXCEPTION: {type(e).__name__}: {e}", flush=True)
    traceback.print_exc()

sim_app.close()
print("[T1] DONE cleanly", flush=True)
