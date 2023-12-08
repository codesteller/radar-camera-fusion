#
 # @ Author: Pallab Maji
 # @ Create Time: 2023-12-08 11:40:18
 # @ Modified time: 2023-12-08 11:42:58
 # @ Description: Enter description here
#

FROM nvcr.io/nvidia/tensorrt:23.09-py3

ENV TZ=Asia/Kolkata
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

USER root
WORKDIR /workspace

RUN apt-get update -y && apt-get upgrade -y
RUN apt-get install  qtbase5-dev qt5-qmake qtcreator \
    qt5-doc qt5-doc-html qtbase5-doc-html qtbase5-examples qtmultimedia5-dev -y
RUN apt update && apt-get install -y python-dev python-numpy \
  python3 python3-pip python3-dev libtbb2 libtbb-dev \
  libjpeg-dev libjasper-dev libdc1394-22-dev \
  python-opencv libopencv-dev libav-tools python-pycurl \
  libatlas-base-dev gfortran webp qt5-default libvtk6-dev zlib1g-dev
RUN apt update && apt install -y cmake g++ wget unzip git
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
