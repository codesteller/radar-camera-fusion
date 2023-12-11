#
 # @ Author: Pallab Maji
 # @ Create Time: 2023-12-08 11:40:18
 # @ Modified time: 2023-12-08 11:42:58
 # @ Description: Enter description here
#

FROM nvcr.io/nvidia/tensorrt:23.09-py3

ARG OPENCV_VERSION=4.8.1


ENV TZ=Asia/Kolkata
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

USER root
WORKDIR /workspace

RUN apt-get update -y && apt-get upgrade -y
RUN apt-get install  qtbase5-dev qt5-qmake qtcreator \
    qt5-doc qt5-doc-html qtbase5-doc-html qtbase5-examples qtmultimedia5-dev -y
RUN apt-get update && apt install -y cmake g++ wget unzip git
RUN apt-get install protobuf-compiler libprotobuf-dev -y

RUN apt-get update -y && apt install -y v4l-utils fswebcam libopencv-dev python3-opencv
RUN apt-get install -y libgtk-3-dev libcanberra-gtk3-module
RUN git clone https://github.com/jbeder/yaml-cpp.git \
    && cd yaml-cpp \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install

RUN apt install libeigen3-dev libtbb-dev libboost-all-dev -y

COPY ./assets/opencv-files/ /workspace/opencv-files/

# Install OpenCV
RUN cd /opt/ &&\
    # Download and unzip OpenCV and opencv_contrib and delte zip files
    wget https://github.com/opencv/opencv/archive/$OPENCV_VERSION.zip -O opencv-${OPENCV_VERSION}.zip &&\
    unzip opencv-${OPENCV_VERSION}.zip &&\
    rm opencv-${OPENCV_VERSION}.zip &&\
    wget https://github.com/opencv/opencv_contrib/archive/$OPENCV_VERSION.zip -O opencv_contrib-${OPENCV_VERSION}.zip &&\
    unzip opencv_contrib-${OPENCV_VERSION}.zip &&\
    rm opencv_contrib-${OPENCV_VERSION}.zip &&\
    # Create build folder and switch to it
    mkdir -p /opt/opencv-${OPENCV_VERSION}/build && cd /opt/opencv-${OPENCV_VERSION}/build &&\
    # Make changes to the OpenCV source files
    cp -r /workspace/opencv-files/normalize_bbox.hpp /opt/opencv-${OPENCV_VERSION}/modules/dnn/src/cuda4dnn/primitives/normalize_bbox.hpp &&\
    cp -r /workspace/opencv-files/region.hpp /opt/opencv-${OPENCV_VERSION}/modules/dnn/src/cuda4dnn/primitives/region.hpp &&\
    # Cmake configure
    cmake \
        -DOPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib-${OPENCV_VERSION}/modules \
        -DWITH_CUDA=ON \
        -DCMAKE_BUILD_TYPE=RELEASE \
        # Install path will be /usr/local/lib (lib is implicit)
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        .. &&\
    # Make
    make -j"$(nproc)" && \
    # Install to /usr/local/lib
    make install && \
    ldconfig 

# Remove OpenCV sources and build folder
# RUN rm -rf /opt/opencv-${OPENCV_VERSION} && rm -rf /opt/opencv_contrib-${OPENCV_VERSION}
