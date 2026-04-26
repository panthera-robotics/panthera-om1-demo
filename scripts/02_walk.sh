#!/bin/bash
# Panthera demo — walk control
# Send a cmd_vel command to Go2.
# Usage:
#   ./02_walk.sh forward   # walk forward 0.4 m/s
#   ./02_walk.sh back      # walk back -0.3 m/s
#   ./02_walk.sh turn      # turn in place 0.5 rad/s
#   ./02_walk.sh circle    # walk in slow circle
#   ./02_walk.sh stop      # stop publishing (Go2 stops after timeout)

ACTION="${1:-forward}"

case "$ACTION" in
  forward)  TWIST='{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {z: 0.0}}'
            DESC="walking forward at 0.4 m/s" ;;
  back)     TWIST='{linear: {x: -0.3, y: 0.0, z: 0.0}, angular: {z: 0.0}}'
            DESC="walking backward at 0.3 m/s" ;;
  turn)     TWIST='{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.5}}'
            DESC="turning in place at 0.5 rad/s" ;;
  circle)   TWIST='{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {z: 0.5}}'
            DESC="walking in a circle (forward + turn)" ;;
  stop)     echo "Stopping publisher..."
            docker exec cmd_vel_publisher pkill -f "topic pub" 2>/dev/null
            sleep 1
            echo "Done. Go2 will stop walking within 1 sec (cmd_vel timeout)."
            exit 0 ;;
  *)        echo "Usage: $0 [forward|back|turn|circle|stop]"
            exit 1 ;;
esac

echo "Stopping any existing publisher..."
docker exec cmd_vel_publisher pkill -f "topic pub" 2>/dev/null
sleep 1

echo "Publishing: Go2 $DESC..."
docker exec -d cmd_vel_publisher bash -c "
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
ros2 topic pub /cmd_vel geometry_msgs/Twist '$TWIST' -r 10
"

sleep 2
echo "Publisher status:"
docker exec cmd_vel_publisher pgrep -af "topic pub" | head -1

echo ""
echo " 👀 Look at VNC — Go2 is now $DESC."
echo " Run with no arg to switch action, or './02_walk.sh stop' to stop."
