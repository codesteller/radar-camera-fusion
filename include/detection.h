/**
 * @ Author: Pallab Maji
 * @ Create Time: 2023-11-20 14:32:08
 * @ Modified time: 2023-12-08 10:58:01
 * @ Description: Enter description here
 */

#ifndef DETECTION_H
#define DETECTION_H

#include <iostream>
#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <oneapi/tbb/concurrent_queue.h>
#include <opencv2/opencv.hpp>
#include "yolov8.hpp"
#include "opencv2/highgui.hpp"
#include "utils.hpp"

namespace adas
{

    const std::vector<std::string> CLASS_NAMES = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
        "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
        "bird", "cat", "dog", "horse", "sheep", "cow", "elephant",
        "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie",
        "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
        "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
        "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
        "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
        "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv",
        "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
        "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
        "teddy bear", "hair drier", "toothbrush"};

    const std::vector<std::vector<unsigned int>> COLORS = {
        {0, 114, 189}, {217, 83, 25}, {237, 177, 32}, {126, 47, 142}, {119, 172, 48}, 
        {77, 190, 238}, {162, 20, 47}, {76, 76, 76}, {153, 153, 153}, {255, 0, 0}, 
        {255, 128, 0}, {191, 191, 0}, {0, 255, 0}, {0, 0, 255}, {170, 0, 255}, 
        {85, 85, 0}, {85, 170, 0}, {85, 255, 0}, {170, 85, 0}, {170, 170, 0}, 
        {170, 255, 0}, {255, 85, 0}, {255, 170, 0}, {255, 255, 0}, {0, 85, 128}, 
        {0, 170, 128}, {0, 255, 128}, {85, 0, 128}, {85, 85, 128}, {85, 170, 128}, 
        {85, 255, 128}, {170, 0, 128}, {170, 85, 128}, {170, 170, 128}, {170, 255, 128}, 
        {255, 0, 128}, {255, 85, 128}, {255, 170, 128}, {255, 255, 128}, {0, 85, 255}, 
        {0, 170, 255}, {0, 255, 255}, {85, 0, 255}, {85, 85, 255}, {85, 170, 255}, 
        {85, 255, 255}, {170, 0, 255}, {170, 85, 255}, {170, 170, 255}, {170, 255, 255}, 
        {255, 0, 255}, {255, 85, 255}, {255, 170, 255}, {85, 0, 0}, {128, 0, 0}, 
        {170, 0, 0}, {212, 0, 0}, {255, 0, 0}, {0, 43, 0}, {0, 85, 0}, {0, 128, 0}, 
        {0, 170, 0}, {0, 212, 0}, {0, 255, 0}, {0, 0, 43}, {0, 0, 85}, {0, 0, 128}, 
        {0, 0, 170}, {0, 0, 212}, {0, 0, 255}, {0, 0, 0}, {36, 36, 36}, {73, 73, 73}, 
        {109, 109, 109}, {146, 146, 146}, {182, 182, 182}, {219, 219, 219}, {0, 114, 189}, 
        {80, 183, 189}, {128, 128, 0}};

    

    class CameraPipeline
    {
    public:
        int m_camera_device;
        cv::VideoCapture m_capture;
        std::string m_window_name_prefix;

        CameraPipeline(const int camera_device, const std::string window_name_prefix);
        CameraPipeline(const int camera_device);
        ~CameraPipeline();

        cv::Mat getFrame();
        void displayFrame();

    private:
        cv::Mat m_frame;
        cv::Mat m_frame_rectified;
        std::string m_window_name;

    }; // class CameraPipeline

    class CameraStreamer
    {
    public:
        // this holds camera stream urls
        std::vector<std::string> camera_source;
        // this holds usb camera indices
        std::vector<int> camera_index;
        // this holds OpenCV VideoCapture pointers
        std::vector<cv::VideoCapture *> camera_capture;
        // this holds queue(s) which hold images from each camera
        std::vector<oneapi::tbb::concurrent_queue<cv::Mat> *> frame_queue;
        // this holds thread(s) which run the camera capture process
        std::vector<std::thread *> camera_thread;
        // this holds the camera configuration file path
        std::vector<std::string> camera_calib_file_paths;
        // Constructor for Camera Device or RTSP URL capture
        CameraStreamer(std::vector<std::string> source);
        // Constructor for USB Camera capture
        CameraStreamer(std::vector<int> index);
        // Constructor for USB Camera capture with calibration file
        CameraStreamer(std::vector<int> index, std::vector<std::string> camera_calib_file_path);
        // Destructor for releasing resource(s)
        ~CameraStreamer();
        

    private:
        bool isUSBCamera;
        bool doRectify;
        int camera_count;
        // initialize and start the camera capturing process(es)
        void startMultiCapture();
        // release all camera capture resource(s)
        void stopMultiCapture();
        // main camera capturing process which will be done by the thread(s)
        void captureFrame(int index);
        // Camera Frame Rectifier
        void load_calibration(void);
        void rectifyFrame(void);

    }; // class CameraStreamer

} // namespace adas

#endif // DETECTION_H