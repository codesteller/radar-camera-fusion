/**
 * @ Author: Pallab Maji
 * @ Create Time: 2023-11-20 15:52:48
 * @ Modified time: 2023-11-22 11:16:36
 * @ Description: Enter description here
 */
#include "main.h"
#include "detection.cpp"
#include <future>
#include <tbb/tbb.h>

int main(int argc, char **argv)
{
    cudaSetDevice(0);

    std::string engine_file_path;
    std::vector<std::string> camera_device;

    if (NUM_CAMERA == 2)
    {
        assert(argc == 4);
        engine_file_path = argv[1];
        camera_device.push_back(argv[2]);
        camera_device.push_back(argv[3]);
    }
    else if (NUM_CAMERA == 1)
    {
        assert(argc == 3);
        engine_file_path = argv[1];
        camera_device.push_back(argv[2]);
    }
    else
    {
        if (LOG_DEBUG_FLAG)
        {
            if (argc < 3)
            {
                std::cout << "argc: " << argc << std::endl;
                for (int i = 0; i < argc; ++i)
                {
                    std::cout << "argv[" << i << "]: " << argv[i] << std::endl;
                }
                std::cout << "Usage: ./adas_fusion <engine_file_path> <camera devices>" << std::endl;
                std::abort();
            }
        }
        else
        {
            std::cout << "Invalid number of camera devices" << std::endl;
            std::cout << "Usage: ./adas_fusion <engine_file_path> <camera devices>" << std::endl;
            std::abort();
        }
    }

    if (LOG_DEBUG_FLAG)
    {
        std::cout << "Model File Path: " << engine_file_path << std::endl;
        std::cout << "Camera Device: " << camera_device[0] << std::endl;
        if (NUM_CAMERA == 2)
            std::cout << "Camera Device: " << camera_device[1] << std::endl;
    }

    std::vector<adas::CameraPipeline> cameraPipelines;

    for (int i = 0; i < NUM_CAMERA; ++i)
    {
        cameraPipelines.push_back(adas::CameraPipeline(camera_device[i]));
    }

    // std::thread thread_obj(cameraPipelines[0].getFrame());

    

    // std::thread t0([&cameraPipelines, &engine_file_path]() {
    //     auto frame = cameraPipelines[0].getFrame();
    //     cameraPipelines[0].displayFrame();
    // });

    // std::thread t1([&cameraPipelines, &engine_file_path]() {
    //     auto frame = cameraPipelines[1].getFrame();
    //     // cameraPipelines[1].displayFrame();
    // });

    // t0.join();
    // t1.join();

    oneapi::tbb::task_group tg;

    tg.run([&cameraPipelines, &engine_file_path]()
           {
               while (true)
               {
                   auto frame_0 = cameraPipelines[0].getFrame();
                   // cameraPipelines[0].displayFrame();
                   cv::imshow("frame_0", frame_0);
                   if (cv::waitKey(1) == 27)
                   {
                       break;
                   }
               } });

    tg.run([&cameraPipelines, &engine_file_path]()
           {
        
            while (true)
            {
                auto frame_1 = cameraPipelines[1].getFrame();
                // cameraPipelines[0].displayFrame();
                // cv::imshow("frame_1", frame_1);
                if (cv::waitKey(1) == 27)
                {
                    break;
                }
            } });

    // tg.wait();

    return 0;
}


// #include "main.h"
// #include "detection.cpp"

// int main()
// {
//     std::vector<int> camera_index = {0, 2};
//     std::vector<std::string> camera_device = {"/dev/video0", "/dev/video2"};

//     std::vector<std::string> label;

//     adas::CameraStreamer cam(camera_index);

//     while (cv::waitKey(20) != 27)
//     {
//         // Retrieve frames from each camera capture thread
//         for (int i = 0; i < camera_index.size(); i++)
//         {
//             cv::Mat frame;
//             // Pop frame from queue and check if the frame is valid
//             if (cam.frame_queue[i]->try_pop(frame))
//             {
//                 // Show frame on Highgui window
//                 cv::imshow(label[i], frame);
//             }
//         }
//     }
// }