#!/bin/bash
device="$1"
rebuild="$2"
if [[ "$(docker images -q docker-swiftly:latest 2> /dev/null)" == "" ]]; then
  # build
    echo "docker-swiftly:latest not found. Going to build the image. This will take time..... Grab a Coffee !!!!"
    docker build -f docker/Dockerfile -t docker-swiftly:latest .
else
    echo "docker-swiftly:latest found"
fi

if [[ "$rebuild" == "rebuild" ]]; then
    echo "Going to rebuild the the image. This will take time..... Grab a Coffee !!!!"
    docker build -f docker/Dockerfile -t docker-swiftly:latest .
fi

# Build The Code
docker run -it --rm --gpus all -v ${PWD}:/workspace/swiftly/ --net=host --ipc=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY --device ${device} \
    -w /workspace/swiftly/build docker-swiftly:latest \
    cmake ..

docker run -it --rm --gpus all -v ${PWD}:/workspace/swiftly/ --net=host --ipc=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY --device ${device} \
    -w /workspace/swiftly/build docker-swiftly:latest \
    make -j$(nproc)

xhost +

docker run -it --rm --gpus all -v ${PWD}:/workspace/swiftly/ --net=host --ipc=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY --device ${device} \
    -w /workspace/swiftly/build docker-swiftly:latest \
    ./adas_fusion ../assets/models/yolov8s.engine ${device}

