#include "main.h"
#include "detection.cpp"

int main()
{
    std::vector<int> camera_index = {0, 2};
    std::vector<std::string> camera_device = {"/dev/video0"};

    std::vector<std::string> label;

    adas::CameraStreamer cam(camera_index);

    while (cv::waitKey(20) != 27)
    {
        // Retrieve frames from each camera capture thread
        for (int i = 0; i < camera_index.size(); i++)
        {
            cv::Mat frame;
            // Pop frame from queue and check if the frame is valid
            if (cam.frame_queue[i]->try_pop(frame))
            {
                // Show frame on Highgui window
                cv::imshow(label[i], frame);
            }
        }
    }
}