/**
 * @ Author: Pallab Maji
 * @ Create Time: 2023-11-20 15:29:23
 * @ Modified time: 2023-11-22 11:05:12
 * @ Description: Enter description here
 */
#include "detection.h"

adas::CameraPipeline::CameraPipeline(const std::string camera_device) : m_camera_device(camera_device)
{
    this->m_capture = cv::VideoCapture(m_camera_device);
    if (!m_capture.isOpened())
    {
        std::cout << "Failed to open camera: " << this->m_camera_device << std::endl;
        std::abort();
    }
    this->m_window_name = "Visualize Input: " + this->m_camera_device;
}

adas::CameraPipeline::CameraPipeline(const std::string camera_device, const std::string window_name_prefix) : m_camera_device{camera_device}, m_window_name_prefix{window_name_prefix}
{
    this->m_window_name_prefix = window_name_prefix;
    this->m_camera_device = camera_device;
    this->m_capture = cv::VideoCapture(m_camera_device);
    if (!m_capture.isOpened())
    {
        std::cout << "Failed to open camera: " << this->m_camera_device << std::endl;
        std::abort();
    }

    this->m_window_name = "Visualize Input: " + this->m_camera_device;
}

adas::CameraPipeline::~CameraPipeline()
{
    this->m_capture.release();
}

cv::Mat adas::CameraPipeline::getFrame()
{
    this->m_capture >> this->m_frame;
    return this->m_frame;
}

void adas::CameraPipeline::displayFrame()
{
    std::cout << "Displaying frame from camera: " << this->m_camera_device << std::endl;
    while (true)
    {
        this->getFrame();
        cv::imshow(this->m_window_name, this->m_frame);
        if (cv::waitKey(1) == 27)
        {
            break;
        }
    }
}

adas::CameraStreamer::CameraStreamer(std::vector<std::string> stream_source)
{
    camera_source = stream_source;
    camera_count = camera_source.size();
    isUSBCamera = false;

    startMultiCapture();
}

adas::CameraStreamer::CameraStreamer(std::vector<int> capture_index)
{
    camera_index = capture_index;
    camera_count = capture_index.size();
    isUSBCamera = true;

    startMultiCapture();
}

adas::CameraStreamer::~CameraStreamer()
{
    stopMultiCapture();
}

void adas::CameraStreamer::captureFrame(int index)
{
    cv::VideoCapture *capture = camera_capture[index];
    while (true)
    {
        cv::Mat frame;
        // Grab frame from camera capture
        (*capture) >> frame;
        // Put frame to the queuestring
        frame_queue[index]->push(frame);
        // relase frame resource
        frame.release();
    }
}

void adas::CameraStreamer::startMultiCapture()
{
    cv::VideoCapture *capture;
    std::thread *t;
    oneapi::tbb::concurrent_queue<cv::Mat> *q;
    std::string camera_device;
    for (int i = 0; i < camera_count; i++)
    {
        // Make VideoCapture instance
        if (!isUSBCamera)
        {
            std::string url = camera_source[i];
            capture = new cv::VideoCapture(url);
            std::cout << "Camera Setup: " << url << std::endl;
        }
        else
        {
            int idx = camera_index[i];
            if (idx < 0)
            {
                std::cout << "Invalid camera index: " << idx << std::endl;
                std::abort();
            }
            else 
            {
                std::cout << "Camera index: " << idx << std::endl;
                camera_device = "/dev/video" + std::to_string(idx);
            }

            // capture = new cv::VideoCapture(idx);
            capture = new cv::VideoCapture(camera_device);
            std::cout << "Camera Setup: " << camera_device << std::endl;
        }

        // Put VideoCapture to the vector
        camera_capture.push_back(capture);

        // Make thread instance
        t = new std::thread(&CameraStreamer::captureFrame, this, i);

        // Put thread to the vector
        camera_thread.push_back(t);

        // Make a queue instance
        q = new oneapi::tbb::concurrent_queue<cv::Mat>;

        // Put queue to the vector
        frame_queue.push_back(q);
    }
}

void adas::CameraStreamer::stopMultiCapture()
{
    cv::VideoCapture *cap;
    for (int i = 0; i < camera_count; i++)
    {
        cap = camera_capture[i];
        if (cap->isOpened())
        {
            // Relase VideoCapture resource
            cap->release();
            std::cout << "Capture " << i << " released" << std::endl;
        }
    }
}
