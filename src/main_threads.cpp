/**
 * @ Author: Pallab Maji
 * @ Create Time: 2023-11-09 11:27:21
 * @ Modified time: 2023-12-04 21:05:47
 * @ Description: Enter description here
 */

#include <tbb/tbb.h>

#include <future>

#include "detection.cpp"
#include "utils.hpp"

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
  std::vector<adas::CameraPipeline> cameraPipelines;
  for (int i = 0; i < camera_devices.size(); i++) {
    label.push_back("Camera Device - " + std::to_string(i));
    cameraPipelines.push_back(
        adas::CameraPipeline(camera_devices[i], label[i]));
  }
  
  
  

  return 0;
}
