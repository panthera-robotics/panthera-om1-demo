#!/bin/bash
# Panthera demo — full shutdown
# Stops everything cleanly. Containers persist (just stopped) so the next
# 01_start_demo.sh is faster (caches still warm).

echo "==============================================="
echo " Panthera demo — shutdown"
echo "==============================================="
echo ""

echo "[1/3] Stopping cmd_vel publisher..."
docker exec cmd_vel_publisher pkill -f "topic pub" 2>/dev/null || true

echo ""
echo "[2/3] Stopping run.py inside Isaac Sim..."
docker exec panthera_om1_demo bash -c 'pkill -9 -f run.py 2>/dev/null; sleep 3' || true

echo ""
echo "[3/3] Stopping containers..."
docker stop cmd_vel_publisher panthera_om1_demo > /dev/null 2>&1

echo ""
echo "Final state:"
docker ps --format "table {{.Names}}\t{{.Status}}"
echo ""
echo "GPU after shutdown:"
nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader
echo ""
echo "==============================================="
echo " Shutdown complete."
echo "==============================================="
echo ""
echo " To start again:    ~/panthera/om1_demo/scripts/01_start_demo.sh"
echo " To stop Vast.ai instance (saves $\$$):  use the Vast dashboard."
echo ""
