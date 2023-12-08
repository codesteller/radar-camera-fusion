# Radar Camera Fusion with TI AWR1843Boost Radar and a USB Camera

## ToDo
- [x] Video streaming from USB Camera 
- [x] Multi-Camera streaming from multiple USB Cameras (Multi Threaded)
- [x] Object detection with Yolo V8s  
- [x] tracking using OpenCV and ByteTracker
- [x] Video BEV Projection
- [x] Docker Build System
- [ ] Interfacing Radar with TI AWR1843Boost Radar SDK
- [ ] Object detection and tracking using Radar
- [ ] Radar-Video Soft Synchronization
- [ ] Radar-Video Calibration
- [ ] Radar-Video Fusion
- [ ] Object detection refactoring using classes
- [ ] Multi Threaded Optimization

## Overview 
This code implements the fusion of radar data from the TI AWR1843Boost Radar and video data from a USB camera. The radar data is processed to detect and track objects in the environment, while the video data is used to provide visual information about the detected objects. The fusion of radar and video data enhances the accuracy and reliability of object detection and tracking.

The code utilizes the TI AWR1843Boost Radar SDK to interface with the radar sensor and obtain raw radar data. The radar data is then processed using signal processing techniques to extract relevant information such as object position, velocity, and size. The USB camera is accessed using OpenCV library to capture video frames. The captured video frames are processed using computer vision algorithms to detect and track objects.

The fusion of radar and video data is achieved by associating the detected objects from radar data with the corresponding objects detected in the video frames. This association is based on the spatial and temporal information of the objects. The fused data is then used for further analysis and decision-making tasks.

This code provides a basic implementation of radar-camera fusion and can be extended and customized for specific applications and requirements. It serves as a starting point for developing more advanced radar-camera fusion systems for various applications such as autonomous driving, surveillance, and robotics.

## Installation
### NGC Container
We are usingf TensorRT container from NGC. The container can be pulled from NGC using the following steps:
* Login to NGC and setup NGC account using you nvidia developers email id.
```bash
docker login nvcr.io
docker pull tensorrt-ubuntu22.04-cuda11.8-cudnn8.2.4.15-runtime:latest
```

### Dependencies
* [OpenCV](https://opencv.org/)
* [TensorRT 8.6](https://developer.nvidia.com/tensorrt)
* [CUDA 11.8](https://developer.nvidia.com/cuda-toolkit)
* [cuDNN 8.9.4](https://developer.nvidia.com/cudnn)
* [Qt 5.15.2](https://www.qt.io/)
* [TI AWR1843Boost Radar SDK](https://www.ti.com/tool/AWR1843BOOST)
* [Yolo V8](https://github.com/ultralytics/ultralytics.git)
* [oneTBB](https://github.com/oneapi-src/oneTBB)
* [YAML-CPP](https://github.com/jbeder/yaml-cpp.git)


## Usage
* Git Clone the repository
* Run the start demo
  * This will build the docker container and run the demo inside it
```
chmod +x startup.sh
./startup.sh
```  


