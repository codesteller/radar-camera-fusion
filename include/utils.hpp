/**
 * @ Author: Pallab Maji
 * @ Create Time: 2023-11-20 13:57:59
 * @ Modified time: 2023-12-04 15:44:09
 * @ Description: Enter description here
 */

#pragma once

#include "detection.h"
#include <chrono>
#include <thread>

#define LOG_DEBUG_FLAG 1
// #define NUM_CAMERA 2

namespace adas {
int check_args(int argc, char **argv) {

  if (argc < 3 || argc > 10) {
    if (LOG_DEBUG_FLAG) {
      std::cout << "argc: " << argc << std::endl;
      for (int i = 0; i < argc; ++i) {
        std::cout << "argv[" << i << "]: " << argv[i] << std::endl;
      }
    }
    std::cout << "[ERROR] Usage: ./adas_fusion \n < camera device 0 > "
                 "<corresponding calibration filepaths> <engine_file_path> "
                 "\n<camera device 1> <corresponding calibration filepaths> "
                 "<engine_file_path> \n <camera device 2> <corresponding "
                 "calibration filepaths> <engine_file_path> "
              << std::endl;
    std::cout << "[NOTE] If no calibration file, enter NA. Maximum 3 Cameras "
                 "Supported"
              << std::endl;
    std::abort();
  }
  return 0;
} // check_args

int get_num_cameras(int argc, char **argv) {
  int num_cameras = (argc - 1) / 3;
  if (num_cameras > 3) {
    std::cout << "[ERROR] Maximum 3 Cameras Supported" << std::endl;
    std::abort();
  }
  if (LOG_DEBUG_FLAG) {
    std::cout << "Number of Cameras: " << num_cameras << std::endl;
    for (int i = 0; i < argc; ++i) {
      std::cout << "argv[" << i << "]: " << argv[i] << std::endl;
    }
  }
  return num_cameras;
} // get_num_cameras

void get_camera_pipelines(int argc, char **argv,
                          std::vector<int> *camera_devices,
                          std::vector<std::string> *calibration_file_paths,
                          std::vector<std::string> *engine_file_paths) {
  check_args(argc, argv);

  // Get Number of Cameras
  int num_cameras = (argc - 1) / 3;
  if (num_cameras > 3) {
    std::cout << "[ERROR] Maximum 3 Cameras Supported" << std::endl;
    std::abort();
  }

  // Get Engine File Paths
  for (int i = 3; i < argc; i += 3) {
    engine_file_paths->push_back(argv[i]);
  }

  // Get Calibration File Paths
  for (int i = 2; i < argc; i += 3) {
    calibration_file_paths->push_back(argv[i]);
  }

  // Get Camera Devices
  for (int i = 1; i < argc; i += 3) {
    camera_devices->push_back(std::stoi(argv[i]));
  }

  if (LOG_DEBUG_FLAG) {
    std::cout << "Number of Cameras: " << num_cameras << std::endl;
    for (int i = 0; i < num_cameras; i++) {
      std::cout << "Camera Device: " << (*camera_devices)[i] << std::endl;
      std::cout << "Calibration File Path: " << (*calibration_file_paths)[i]
                << std::endl;
      std::cout << "Engine File Path: " << (*engine_file_paths)[i] << std::endl;
    }
  }

} // get_camera_pipelines

} // namespace adas