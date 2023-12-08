/**
 * @ Author: Pallab Maji
 * @ Create Time: 2023-11-20 13:57:59
 * @ Modified time: 2023-12-07 20:40:24
 * @ Description: Enter description here
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <thread>

#include "ByteTrack/BYTETracker.h"
#include "ByteTrack/Object.h"
#include "ByteTrack/Rect.h"
#include "common.hpp"
#include "detection.h"
#include "opencv2/opencv.hpp"
#include "BEVTransform/camera_model.h"

#define LOG_DEBUG_FLAG 1
// #define NUM_CAMERA 2

namespace adas {

struct calib_params {
  int image_width;
  int image_height;
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
};

// get calibration parameters from yaml file

struct calib_params get_calibration_params(std::string config_file) {
  YAML::Node config = YAML::LoadFile(config_file);

  calib_params calib;
  calib.image_width = config["image_width"].as<int>();
  calib.image_height = config["image_height"].as<int>();
  calib.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  calib.camera_matrix.at<double>(0, 0) =
      config["camera_matrix"]["data"][0].as<double>();
  calib.camera_matrix.at<double>(0, 1) =
      config["camera_matrix"]["data"][1].as<double>();
  calib.camera_matrix.at<double>(0, 2) =
      config["camera_matrix"]["data"][2].as<double>();
  calib.camera_matrix.at<double>(1, 0) =
      config["camera_matrix"]["data"][3].as<double>();
  calib.camera_matrix.at<double>(1, 1) =
      config["camera_matrix"]["data"][4].as<double>();
  calib.camera_matrix.at<double>(1, 2) =
      config["camera_matrix"]["data"][5].as<double>();
  calib.camera_matrix.at<double>(2, 0) =
      config["camera_matrix"]["data"][6].as<double>();
  calib.camera_matrix.at<double>(2, 1) =
      config["camera_matrix"]["data"][7].as<double>();
  calib.camera_matrix.at<double>(2, 2) =
      config["camera_matrix"]["data"][8].as<double>();
  calib.dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
  calib.dist_coeffs.at<double>(0, 0) =
      config["distortion_coefficients"]["data"][0].as<double>();
  calib.dist_coeffs.at<double>(1, 0) =
      config["distortion_coefficients"]["data"][1].as<double>();
  calib.dist_coeffs.at<double>(2, 0) =
      config["distortion_coefficients"]["data"][2].as<double>();
  calib.dist_coeffs.at<double>(3, 0) =
      config["distortion_coefficients"]["data"][3].as<double>();
  calib.dist_coeffs.at<double>(4, 0) =
      config["distortion_coefficients"]["data"][4].as<double>();

  return calib;
}

// draw the tracked object on the image

void draw_tracked_objects(
    cv::Mat &image, cv::Mat &dest_image,
    const std::vector<byte_track::BYTETracker::STrackPtr> &tracked_objects,
    const std::vector<det::Object> &detected_objs,
    const std::vector<cv::String> CLASS_NAMES,
    std::vector<cvflann::lsh::Bucket> COLORS,
    std::vector<float> distances) {
  dest_image = image.clone();
  unsigned int idx = 0;

  std::cout << "tracked_objects.size(): " << tracked_objects.size() << std::endl;
  std::cout << "detected_obj.size(): " << detected_objs.size() << std::endl;
  
  std::string class_name = "Object";
  for (auto &obj : tracked_objects) {
    const auto &rect = obj->getRect();
    const auto &track_id = obj->getTrackId();
    const auto &state = obj->getSTrackState();

    // TODO fix the Class names correctly in the tracker
    // find class name
    det::Object curr_obj = detected_objs[idx];
    class_name = CLASS_NAMES[curr_obj.label];
    cv::Scalar color = cv::Scalar(COLORS[curr_obj.label][0], COLORS[curr_obj.label][1], COLORS[curr_obj.label][2]);

    // if (state == byte_track::STrackState::Tracked) {
    //   cv::rectangle(
    //       dest_image, cv::Point(rect.x(), rect.y()),
    //       cv::Point(rect.x() + rect.width(), rect.y() + rect.height()),
    //       color, 2);
    //   cv::putText(dest_image, class_name + std::to_string(track_id),
    //               cv::Point(rect.x(), rect.y()), cv::FONT_HERSHEY_COMPLEX, 1,
    //               cv::Scalar(0, 255, 0), 2);

    if (state == byte_track::STrackState::Tracked) {
      cv::rectangle(
          dest_image, cv::Point(rect.x(), rect.y()),
          cv::Point(rect.x() + rect.width(), rect.y() + rect.height()),
          color, 2);
      cv::putText(dest_image, std::to_string(track_id) + " @ " + std::to_string(distances[idx]) + "m",
                  cv::Point(rect.x(), rect.y()), cv::FONT_HERSHEY_COMPLEX, 1,
                  cv::Scalar(0, 255, 0), 2);
    
    } else if (state == byte_track::STrackState::Lost) {
      std::cout << "Lost Track ID: "<< track_id << std::endl;
    } else if (state == byte_track::STrackState::Removed) {
      std::cout << "Removed Track ID: "<< track_id << std::endl;
    }
    
    idx++;
  }
}

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
}  // check_args

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
}  // get_num_cameras

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

}  // get_camera_pipelines

}  // namespace adas