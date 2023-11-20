# Radar Camera Fusion with TI AWR1843Boost Radar and a USB Camera

## ToDo
- [x] Video streaming from USB Camera 
- [x] Object detection with Yolo V8s 
- [ ] tracking using OpenCV and ByteTracker
- [ ] Interfacing Radar with TI AWR1843Boost Radar SDK
- [ ] Object detection and tracking using Radar
- [ ] Radar-Video Soft Synchronization
- [ ] Video BEV Projection
- [ ] Radar-Video Calibration

## Overview 
This code implements the fusion of radar data from the TI AWR1843Boost Radar and video data from a USB camera. The radar data is processed to detect and track objects in the environment, while the video data is used to provide visual information about the detected objects. The fusion of radar and video data enhances the accuracy and reliability of object detection and tracking.

The code utilizes the TI AWR1843Boost Radar SDK to interface with the radar sensor and obtain raw radar data. The radar data is then processed using signal processing techniques to extract relevant information such as object position, velocity, and size. The USB camera is accessed using OpenCV library to capture video frames. The captured video frames are processed using computer vision algorithms to detect and track objects.

The fusion of radar and video data is achieved by associating the detected objects from radar data with the corresponding objects detected in the video frames. This association is based on the spatial and temporal information of the objects. The fused data is then used for further analysis and decision-making tasks.

This code provides a basic implementation of radar-camera fusion and can be extended and customized for specific applications and requirements. It serves as a starting point for developing more advanced radar-camera fusion systems for various applications such as autonomous driving, surveillance, and robotics.

## Usage


