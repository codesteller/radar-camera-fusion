/**
 * @ Author: Pallab Maji
 * @ Create Time: 2023-11-20 15:29:23
 * @ Modified time: 2023-12-08 10:58:59
 * @ Description: Enter description here
 */
#include "detection.h"

adas::CameraPipeline::CameraPipeline(const int camera_device)
    : m_camera_device(camera_device) {
  this->m_capture = cv::VideoCapture(m_camera_device);
  if (!m_capture.isOpened()) {
    std::cout << "[ERROR] Failed to open camera: " << this->m_camera_device
              << std::endl;
    std::abort();
  }
  this->m_window_name = "Visualize Input: " + this->m_camera_device;
}

adas::CameraPipeline::CameraPipeline(const int camera_device,
                                     const std::string window_name_prefix)
    : m_camera_device{camera_device}, m_window_name_prefix{window_name_prefix} {
  this->m_window_name_prefix = window_name_prefix;
  this->m_camera_device = camera_device;
  this->m_capture = cv::VideoCapture(m_camera_device);
  if (!m_capture.isOpened()) {
    std::cout << "[ERROR] Failed to open camera: " << this->m_camera_device
              << std::endl;
    std::abort();
  }

  this->m_window_name = "Visualize Input: " + this->m_camera_device;
}

adas::CameraPipeline::~CameraPipeline() { this->m_capture.release(); }

cv::Mat adas::CameraPipeline::getFrame() {
  this->m_capture >> this->m_frame;
  return this->m_frame;
}

void adas::CameraPipeline::displayFrame() {
  if (LOG_DEBUG_FLAG)
    std::cout << "Displaying frame from camera: " << this->m_camera_device
              << std::endl;
  while (true) {
    this->getFrame();
    cv::imshow(this->m_window_name, this->m_frame);
    if (cv::waitKey(1) == 27) {
      break;
    }
  }
}

/*
    Camera Streamer Object
*/

adas::CameraStreamer::CameraStreamer(std::vector<std::string> stream_source) {
  camera_source = stream_source;
  camera_count = camera_source.size();
  isUSBCamera = false;

  startMultiCapture();
}

adas::CameraStreamer::CameraStreamer(std::vector<int> capture_index) {
  camera_index = capture_index;
  camera_count = capture_index.size();
  isUSBCamera = true;
  doRectify = false;

  startMultiCapture();
}

adas::CameraStreamer::CameraStreamer(std::vector<int> capture_index,
                                     std::vector<std::string> file_paths) {
  camera_index = capture_index;
  camera_count = capture_index.size();
  isUSBCamera = true;
  this->camera_calib_file_paths = file_paths;
  if (LOG_DEBUG_FLAG) {
    for (int i = 0; i < camera_count; i++) {
      std::cout << "[INFO] Camera Calibration File Path: "
                << this->camera_calib_file_paths[i] << std::endl;
    }
  }
  doRectify = true;

  load_calibration();
  startMultiCapture();
}

adas::CameraStreamer::~CameraStreamer() { stopMultiCapture(); }

void adas::CameraStreamer::captureFrame(int index) {
  cv::VideoCapture *capture = camera_capture[index];
  while (true) {
    cv::Mat frame;
    // Grab frame from camera capture
    (*capture) >> frame;
    // Put frame to the queuestring
    frame_queue[index]->push(frame);
    //
    if (doRectify) {
      // Camera Frame Rectifier
    }
    // release frame resource
    frame.release();
  }
}

void adas::CameraStreamer::startMultiCapture() {
  cv::VideoCapture *capture;
  std::thread *t;
  oneapi::tbb::concurrent_queue<cv::Mat> *q;
  std::string camera_device;
  for (int i = 0; i < camera_count; i++) {
    // Make VideoCapture instance
    if (!isUSBCamera) {
      std::string url = camera_source[i];
      capture = new cv::VideoCapture(url);
      std::cout << "[INFO] Camera Setup: " << url << std::endl;
    } else {
      int idx = camera_index[i];
      if (idx < 0) {
        std::cout << "[INFO] Invalid camera index: " << idx << std::endl;
        std::abort();
      } else {
        std::cout << "[INFO] Camera index: " << idx << std::endl;
        camera_device = "/dev/video" + std::to_string(idx);
      }

      // capture = new cv::VideoCapture(idx);
      // capture = new cv::VideoCapture(camera_device, cv::CAP_V4L2);
      capture = new cv::VideoCapture(camera_device);
      capture->set(cv::CAP_PROP_FRAME_WIDTH, 1920);
      capture->set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
      std::cout << "[INFO] Camera Setup completed for Device ID: " << camera_device << std::endl;
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

void adas::CameraStreamer::load_calibration() {
  std::string path = this->camera_calib_file_paths[0];
  
    // Read Calibration File using OpenCV
    cv::FileStorage cam_settings(path, cv::FileStorage::READ);
    // bool read_ok = cam_settings.isOpened();
    // if (!read_ok) {
    //   std::cout << "Failed to open calibration file: " << path << std::endl;
    //   std::abort();
    // }
    // // Read Camera Matrix
    // cv::Mat camera_matrix;
    // cam_settings["Camera_Matrix"] >> camera_matrix;
    // std::cout << "Camera Matrix: " << camera_matrix << std::endl;
}

void adas::CameraStreamer::stopMultiCapture() {
  cv::VideoCapture *cap;
  for (int i = 0; i < camera_count; i++) {
    cap = camera_capture[i];
    if (cap->isOpened()) {
      // Relase VideoCapture resource
      cap->release();
      std::cout << "Capture " << i << " released" << std::endl;
    }
  }
}
