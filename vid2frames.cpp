// A utility to convert a video to frames using OPENCV.

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>

int main (int argc, char **argv)
{
        if (argc != 3)
        {
                std::cout << "Please use vid2frames <VideoFileName> <Output Path>"<< std::endl;
                return 1;
        }
        else
        {
                cv::VideoCapture cap(argv[1]);
                if (!cap.isOpened())
                {
                        std::cerr << "ERROR: Could not open video " << argv[1] << std::endl;
                        return 1;
                }
                int frame_count = 1;
                while (true)
                {
                        cv::Mat frame;
                        cap >> frame;
                        if (frame.empty())
                        {
                                break;
                        }
                        char outname[128];
                        sprintf(outname,"frame_%06d.jpeg", frame_count);
                        //std::cout << outname << std::endl;
                        cv::imwrite(outname,frame);
                        frame_count++;
                }
        }
        return 0;
}
