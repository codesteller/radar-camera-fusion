/**
 * @ Author: Pallab Maji
 * @ Create Time: 2023-11-20 15:52:48
 * @ Modified time: 2023-12-04 20:15:07
 * @ Description: Enter description here
 */
#include "detection.cpp"
#include "utils.hpp"
#include <future>
#include <tbb/tbb.h>

int main(int argc, char **argv) {
  cudaSetDevice(0);

  std::vector<int> camera_devices;
  std::vector<std::string> calibration_file_paths;
  std::vector<std::string> engine_file_paths;

  // Check if the number of arguments are valid
  adas::get_camera_pipelines(argc, argv, &camera_devices,
                             &calibration_file_paths, &engine_file_paths);

  // Highgui window titles
  std::vector<std::string> label;

  for (int i = 0; i < camera_devices.size(); i++) {
    std::string title = "Camera Device " + std::to_string(i);
    label.push_back(title);
    cv::namedWindow(label[i], cv::WND_PROP_FULLSCREEN);
  }

  adas::CameraStreamer cameraStreamer(camera_devices, calibration_file_paths);
  while (cv::waitKey(1) != 27) {
    // Retrieve frames from each camera capture thread
    for (int i = 0; i < camera_devices.size(); i++) {
      cv::Mat frame;
      // Pop frame from queue and check if the frame is valid
      if (cameraStreamer.frame_queue[i]->try_pop(frame)) {
        // Show frame on Highgui window
        if (LOG_DEBUG_FLAG)
          cv::imshow(label[i], frame);
      }
    }
  }
  return 0;
}
