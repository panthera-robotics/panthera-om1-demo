#!/bin/bash
# Quick test: does X11 from container work? Shows xclock window
echo "DISPLAY: $DISPLAY"
xhost +local:docker 2>&1 | head -1
docker run --rm \
  --user 1234:1234 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --entrypoint bash \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  -c "apt list --installed 2>/dev/null | grep -i x11 | head -3; ls /tmp/.X11-unix/"
