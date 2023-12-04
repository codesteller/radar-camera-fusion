/**
 * @ Author: Pallab Maji
 * @ Create Time: 2023-11-20 15:52:48
 * @ Modified time: 2023-12-04 13:43:02
 * @ Description: Enter description here
 */
#include "main.h"
#include "detection.cpp"
#include <future>
#include <tbb/tbb.h>

int main(int argc, char **argv) {
  cudaSetDevice(0);

  std::string engine_file_path;
  std::vector<int> camera_device;

  if (argc == 3) {
    engine_file_path = argv[1];
    camera_device.push_back(std::stoi(argv[2]));
  } else if (argc == 4) {
    engine_file_path = argv[1];
    camera_device.push_back(std::stoi(argv[2]));
    camera_device.push_back(std::stoi(argv[3]));
  } else {
    if (LOG_DEBUG_FLAG) {
      if (argc < 3) {
        std::cout << "argc: " << argc << std::endl;
        for (int i = 0; i < argc; ++i) {
          std::cout << "argv[" << i << "]: " << argv[i] << std::endl;
        }
        std::cout << "Usage: ./adas_fusion <engine_file_path> <camera devices>"
                  << std::endl;
        std::abort();
      }
    } else {
      std::cout << "Invalid number of camera devices" << std::endl;
      std::cout << "Usage: ./adas_fusion <engine_file_path> <camera devices>"
                << std::endl;
      std::abort();
    }
  }

  if (LOG_DEBUG_FLAG) {
    std::cout << "Model File Path: " << engine_file_path << std::endl;
    std::cout << "Camera Device: " << camera_device[0] << std::endl;
    if (argc == 4)
      std::cout << "Camera Device: " << camera_device[1] << std::endl;
  }

  // Highgui window titles
  std::vector<std::string> label;

  for (int i = 0; i < camera_device.size(); i++) {
    std::string title = "Camera Device " + std::to_string(i);
    label.push_back(title);
    cv::namedWindow(label[i], cv::WND_PROP_FULLSCREEN);
  }

  adas::CameraStreamer cameraStreamer(camera_device);
  while (cv::waitKey(20) != 27) {
    // Retrieve frames from each camera capture thread
    for (int i = 0; i < camera_device.size(); i++) {
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
