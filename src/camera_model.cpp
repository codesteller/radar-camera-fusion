#include "BEVTransform/camera_model.h"

using namespace std;
using namespace cv;

CameraModel::CameraModel() { readCalibFile(SMARTCAM_CAMERA_CALIB_FILE); }

void CameraModel::readCalibFile(std::string file_path) {
  if (!fs::exists(file_path)) {
    return;
  }

  // Read data file
  std::ifstream data_file(file_path);

  float car_width;
  float carpet_width;
  float car_to_carpet_distance;
  float carpet_length;
  float tl_x;
  float tl_y;
  float tr_x;
  float tr_y;
  float br_x;
  float br_y;
  float bl_x;
  float bl_y;

  std::string line;
  data_file >> line >> car_width;
  data_file >> line >> carpet_width;
  data_file >> line >> car_to_carpet_distance;
  data_file >> line >> carpet_length;
  data_file >> line >> tl_x >> line >> tl_y;
  data_file >> line >> tr_x >> line >> tr_y;
  data_file >> line >> br_x >> line >> br_y;
  data_file >> line >> bl_x >> line >> bl_y;

  updateCameraModel(car_width, carpet_width, car_to_carpet_distance,
                    carpet_length, tl_x, tl_y, tr_x, tr_y, br_x, br_y, bl_x,
                    bl_y);
}

void CameraModel::updateCameraModel(float car_width, float carpet_width,
                                    float car_to_carpet_distance,
                                    float carpet_length, float tl_x, float tl_y,
                                    float tr_x, float tr_y, float br_x,
                                    float br_y, float bl_x, float bl_y) {
  std::cout << "car_width " << car_width << std::endl;
  std::cout << "carpet_width " << carpet_width << std::endl;
  std::cout << "car_to_carpet_distance " << car_to_carpet_distance << std::endl;
  std::cout << "carpet_length " << carpet_length << std::endl;
  std::cout << "tl_x " << tl_x << std::endl;
  std::cout << "tl_y " << tl_y << std::endl;
  std::cout << "tr_x " << tr_x << std::endl;
  std::cout << "tr_y " << tr_y << std::endl;
  std::cout << "br_x " << br_x << std::endl;
  std::cout << "br_y " << br_y << std::endl;
  std::cout << "bl_x " << bl_x << std::endl;
  std::cout << "bl_y " << bl_y << std::endl;

  FourPoints four_image_points =
      FourPoints(cv::Point2f(tl_x, tl_y), cv::Point2f(tr_x, tr_y),
                 cv::Point2f(br_x, br_y), cv::Point2f(bl_x, bl_y));
  birdview_model.calibrate(car_width, carpet_width, car_to_carpet_distance,
                           carpet_length, four_image_points);
}

BirdViewModel *CameraModel::getBirdViewModel() { return &birdview_model; }

bool CameraModel::isCalibrated() {
  return true;
  // return birdview_model.isCalibrated();
}