#!/bin/bash
device="$1"
docker build -f docker/Dockerfile -t docker-swiftly:latest .
docker run -it --rm --gpus all -v ${PWD}:/workspace/swiftly/ --net=host --ipc=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY --device ${device} \
    -w /workspace/swiftly/build docker-swiftly:latest \
    ./adas_fusion ../assets/models/yolov8s.engine ${device}