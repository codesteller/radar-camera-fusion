/**
 * @ Author: Pallab Maji
 * @ Create Time: 2023-11-20 13:11:32
 * @ Modified time: 2023-12-07 20:40:33
 * @ Description: Enter description here
 */

#include "utils.hpp"

int main(int argc, char **argv) {
  // cuda:0
  cudaSetDevice(0);
  bool isCamera{false};
  bool isVideo{false};

  if (LOG_DEBUG_FLAG) {
    if (argc < 3) {
      std::cout << "argc: " << argc << std::endl;
      for (int i = 0; i < argc; ++i) {
        std::cout << "argv[" << i << "]: " << argv[i] << std::endl;
      }
      std::cout << "Usage: ./adas_fusion <engine_file_path> <image_path>"
                << std::endl;
      std::abort();
    }
  } else {
    assert(argc == 3);
  }

  const std::string engine_file_path{argv[1]};

  const std::string path{argv[2]};

  if (path.find("/dev/video") != std::string::npos) {
    std::cout << "Using camera: " << path << std::endl;
    isCamera = true;
  } else {
    std::cout << "Using Images or Video: " << path << std::endl;
  }

  std::vector<std::string> imagePathList;

  // Setup the Model
  auto yolov8 = new YOLOv8(engine_file_path);
  yolov8->make_pipe(true);

  // Setup the BEV Camera Model
  // Setup Camera Model
  CameraModel camera_model;
  const auto bev_model = camera_model.getBirdViewModel();

  bev_model->make_bev_matrix();


  if (IsFile(path)) {
    std::string suffix = path.substr(path.find_last_of('.') + 1);
    if (suffix == "jpg" || suffix == "jpeg" || suffix == "png") {
      imagePathList.push_back(path);
    } else if (suffix == "mp4" || suffix == "avi" || suffix == "m4v" ||
               suffix == "mpeg" || suffix == "mov" || suffix == "mkv") {
      isVideo = true;
    } else {
      printf("suffix %s is wrong !!!\n", suffix.c_str());
      std::abort();
    }
  } else if (IsFolder(path)) {
    cv::glob(path + "/*.jpg", imagePathList);
  }

  cv::Mat frame_det, image;
  cv::Size size = cv::Size{640, 640};
  std::vector<Object> objs;

  cv::namedWindow("Input Frames", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Detection Output", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Tracker Output", cv::WINDOW_AUTOSIZE);

  

  if (isVideo) {
    cv::VideoCapture cap(path);

    if (!cap.isOpened()) {
      printf("can not open %s\n", path.c_str());
      return -1;
    }
    while (cap.read(image)) {
      objs.clear();
      yolov8->copy_from_Mat(image, size);
      auto start = std::chrono::system_clock::now();
      yolov8->infer();
      auto end = std::chrono::system_clock::now();
      yolov8->postprocess(objs);
      yolov8->draw_objects(image, frame_det, objs, adas::CLASS_NAMES,
                           adas::COLORS);
      auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(
                    end - start)
                    .count() /
                1000.;
      printf("cost %2.4lf ms\n", tc);
      cv::imshow("Detection Output", frame_det);
      if (cv::waitKey(10) == 'q') {
        break;
      }
    }
  } else if (isCamera) {
    std::cout << "[INFO] Using camera: " << path << std::endl;
    cv::VideoCapture cap(path);
    // if not success, exit program
    if (cap.isOpened() == false) {
      std::cout << "Cannot open the video camera" << path << std::endl;
      std::abort();
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1020);

    double dWidth = cap.get(
        cv::CAP_PROP_FRAME_WIDTH);  // get the width of frames of the video
    double dHeight = cap.get(
        cv::CAP_PROP_FRAME_HEIGHT);  // get the height of frames of the video

    std::cout << "Resolution of the video : " << dWidth << " x " << dHeight
              << std::endl;

    // Load Calibration Parameters
    std::string calibration_file_path =
        "../assets/scooter-calibration-files/BR01FU9650/"
        "BR01FU9650-1920x1020.yaml";
    adas::calib_params calib =
        adas::get_calibration_params(calibration_file_path);

    // TODO: Add multiple Tracker Type
    // Default (0): ByteTracker
    // 1: CSRT
    // 2: KCF
    // int trackerType = 3;

    // Create Tracker
    byte_track::BYTETracker tracker;

    while (cap.read(image)) {
      if (image.empty()) {
        std::cout << "[ERROR] Video camera is disconnected" << std::endl;
        std::abort();
      }

      std::cout << "[INFO] Resolution of the video : " << dWidth << " x "
                << dHeight << std::endl;

      // Undistort the image
      auto start_full = std::chrono::system_clock::now();
      cv::Mat frame_rectified, frame_tracked;
      cv::undistort(image, frame_rectified, calib.camera_matrix,
                    calib.dist_coeffs);

      objs.clear();
      yolov8->copy_from_Mat(frame_rectified, size);
      auto start = std::chrono::system_clock::now();
      yolov8->infer();
      auto end = std::chrono::system_clock::now();
      yolov8->postprocess(objs);
      yolov8->draw_objects(frame_rectified, frame_det, objs, adas::CLASS_NAMES,
                           adas::COLORS);
      auto end_full = std::chrono::system_clock::now();
      auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(
                    end - start)
                    .count() /
                1000.;
      auto tc_full =
          (double)std::chrono::duration_cast<std::chrono::microseconds>(
              end_full - start_full)
              .count() /
          1000.;

      // TODO: [HIGH] Implement one Single Detection Structure
      // Current detector is using YOLO v8 cv:rect based structure and
      // ByteTrack using Eigen based. Need to convert one to another for every
      // frame

      // Convert Objects to STrackPtr
      std::vector<byte_track::Object> objs_st = {};
      for (auto &obj : objs) {
        std::cout << "Object: " << obj.rect.x << " " << obj.rect.y << " "
                  << obj.rect.width << " " << obj.rect.height << " "
                  << obj.label << " " << obj.prob << std::endl;
        byte_track::Object obj_st(
            byte_track::Rect(obj.rect.x, obj.rect.y, obj.rect.width,
                             obj.rect.height),
            obj.label, obj.prob);
        objs_st.push_back(obj_st);
      }

      /*
              TRACKER UPDATE
      */

      // Update tracker
      const auto outputs = tracker.update(objs_st);
      std::vector<float> distances;

      for (auto &output : outputs) {
        
        const auto &rect = output->getRect();
        const auto &track_id = output->getTrackId();
        std::cout << "Object: " << rect.x() << " " << rect.y() << " "
                  << rect.width() << " " << rect.height() << " " << track_id
                  << " "
                  << "1.0" << std::endl;
        // TODO : Refactor later; for testing only

        std::vector<cv::Point2f> points;
        points.push_back(cv::Point2f(
            static_cast<float>(rect.x()) / dWidth, 
            static_cast<float>(rect.y() + rect.height()) / dHeight));
        points.push_back(cv::Point2f(
            static_cast<float>(rect.x() + rect.width()) / dWidth, 
            static_cast<float>(rect.y() + rect.height()) / dHeight));

        std::vector<cv::Point2f> transformed_points;
        bev_model->transformPoints(points, transformed_points);

        float max_y = std::max({transformed_points[0].y,
            transformed_points[1].y
        });

        float distance = bev_model->getDistanceToCar(max_y);
        distances.push_back(distance);

        std::cout << "Distance: " << distance << std::endl;
      }
      

      // Draw Objects on Tracked Frame
      adas::draw_tracked_objects(frame_rectified, frame_tracked, outputs, objs,
                                 adas::CLASS_NAMES, adas::COLORS, distances);

      /*
              Final Inference Time Calculation and Display
      */
      // Format Output Video [Remove this Later]
      std::string str = "Inference Time: " + std::to_string(tc) + " ms";
      std::string str_full =
          "Camera Pipeline Time: " + std::to_string(tc_full) + " ms";

      // Draw Frame Rate on Detection Output
      cv::putText(frame_det, str, cv::Point(50, 50), cv::FONT_HERSHEY_DUPLEX, 1,
                  cv::Scalar(0, 255, 0), 2, false);
      cv::putText(frame_det, str_full, cv::Point(50, 100),
                  cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0), 2, false);

      std::cout << str << std::endl;
      std::cout << str_full << std::endl;
      std::cout << "-------------------------" << std::endl;

      cv::imshow("Detection Output", frame_det);
      cv::imshow("Input Frames", frame_rectified);
      cv::imshow("Tracker Output", frame_tracked);

      std::cout << "-------------------------" << std::endl;
      if (cv::waitKey(1) == 'q') {
        break;
      }
    }
  }  // end Else If isCamera
  else {
    for (auto &path : imagePathList) {
      objs.clear();
      image = cv::imread(path);
      yolov8->copy_from_Mat(image, size);
      auto start = std::chrono::system_clock::now();
      yolov8->infer();
      auto end = std::chrono::system_clock::now();
      yolov8->postprocess(objs);
      yolov8->draw_objects(image, frame_det, objs, adas::CLASS_NAMES,
                           adas::COLORS);
      auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(
                    end - start)
                    .count() /
                1000.;
      printf("cost %2.4lf ms\n", tc);
      cv::imshow("Detection Output", frame_det);
      // cv::waitKey(0);
      if (cv::waitKey(10) == 'q') {
        break;
      }
    }
  }
  cv::destroyAllWindows();
  delete yolov8;
  return 0;
}
