/**
 * @ Author: Pallab Maji
 * @ Create Time: 2023-11-20 13:11:32
 * @ Modified time: 2023-12-05 15:30:01
 * @ Description: Enter description here
 */

#include "utils.hpp"

int main(int argc, char **argv)
{
    // cuda:0
    cudaSetDevice(0);
    bool isCamera{false};
    bool isVideo{false};

    if (LOG_DEBUG_FLAG)
    {
        if (argc < 3)
        {
            std::cout << "argc: " << argc << std::endl;
            for (int i = 0; i < argc; ++i)
            {
                std::cout << "argv[" << i << "]: " << argv[i] << std::endl;
            }
            std::cout << "Usage: ./yolov8 <engine_file_path> <image_path>" << std::endl;
            std::abort();
        }
    }
    else
    {
        assert(argc == 3);
    }

    const std::string engine_file_path{argv[1]};

    const std::string path{argv[2]};

    if (path == "/dev/video0")
    {
        std::cout << "Using camera: " << path << std::endl;
        isCamera = true;
    }
    else
    {
        std::cout << "Using Images or Video: " << path << std::endl;
    }

    std::vector<std::string> imagePathList;

    auto yolov8 = new YOLOv8(engine_file_path);
    yolov8->make_pipe(true);

    if (IsFile(path))
    {
        std::string suffix = path.substr(path.find_last_of('.') + 1);
        if (suffix == "jpg" || suffix == "jpeg" || suffix == "png")
        {
            imagePathList.push_back(path);
        }
        else if (suffix == "mp4" || suffix == "avi" || suffix == "m4v" || suffix == "mpeg" || suffix == "mov" || suffix == "mkv")
        {
            isVideo = true;
        }
        else
        {
            printf("suffix %s is wrong !!!\n", suffix.c_str());
            std::abort();
        }
    }
    else if (IsFolder(path))
    {
        cv::glob(path + "/*.jpg", imagePathList);
    }

    cv::Mat res, image;
    cv::Size size = cv::Size{640, 640};
    std::vector<Object> objs;

    cv::namedWindow("Input Frames", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Detection Output", cv::WINDOW_AUTOSIZE);

    if (isVideo)
    {
        cv::VideoCapture cap(path);

        if (!cap.isOpened())
        {
            printf("can not open %s\n", path.c_str());
            return -1;
        }
        while (cap.read(image))
        {
            objs.clear();
            yolov8->copy_from_Mat(image, size);
            auto start = std::chrono::system_clock::now();
            yolov8->infer();
            auto end = std::chrono::system_clock::now();
            yolov8->postprocess(objs);
            yolov8->draw_objects(image, res, objs, adas::CLASS_NAMES, adas::COLORS);
            auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;
            printf("cost %2.4lf ms\n", tc);
            cv::imshow("Detection Output", res);
            if (cv::waitKey(10) == 'q')
            {
                break;
            }
        }
    }
    else if (isCamera)
    {
        cv::VideoCapture cap(path);
        // if not success, exit program
        if (cap.isOpened() == false)
        {
            std::cout << "Cannot open the video camera" << path << std::endl;
            std::abort();
        }

        double dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);   // get the width of frames of the video
        double dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT); // get the height of frames of the video

        std::cout << "Resolution of the video : " << dWidth << " x " << dHeight << std::endl;

        while (cap.read(image))
        {
            objs.clear();
            yolov8->copy_from_Mat(image, size);
            auto start = std::chrono::system_clock::now();
            yolov8->infer();
            auto end = std::chrono::system_clock::now();
            yolov8->postprocess(objs);
            yolov8->draw_objects(image, res, objs, adas::CLASS_NAMES, adas::COLORS);
            auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;
            printf("cost %2.4lf ms\n", tc);
            cv::imshow("Detection Output", res);
            if (cv::waitKey(10) == 'q')
            {
                break;
            }
        }
    } // end Else If isCamera
    else
    {
        for (auto &path : imagePathList)
        {
            objs.clear();
            image = cv::imread(path);
            yolov8->copy_from_Mat(image, size);
            auto start = std::chrono::system_clock::now();
            yolov8->infer();
            auto end = std::chrono::system_clock::now();
            yolov8->postprocess(objs);
            yolov8->draw_objects(image, res, objs, adas::CLASS_NAMES, adas::COLORS);
            auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;
            printf("cost %2.4lf ms\n", tc);
            cv::imshow("Detection Output", res);
            // cv::waitKey(0);
            if (cv::waitKey(10) == 'q')
            {
                break;
            }
        }
    }
    cv::destroyAllWindows();
    delete yolov8;
    return 0;
}
