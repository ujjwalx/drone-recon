// A utility to convert a video to frames using OPENCV.
// Ujjwal Sharma

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>

int main (int argc, char **argv)
{
        if (argc != 4)
        {
                std::cout << "Please use vid2frames <VideoFileName> <Output Path> <frame_skip_count>"<< std::endl;
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
                int count = 1;
                int frame_skip = atoi(argv[3]);
                while (true)
                {
                        cv::Mat frame;
                        cap >> frame;
                        if (frame.empty())
                        {
                                break;
                        }
                        if (count % frame_skip == 0) {
                                char outname[128];
                                sprintf(outname,"%s/frame_%06d.jpeg",argv[2],frame_count);
                                //std::cout << outname << std::endl;
                                cv::imwrite(outname,frame);
                                frame_count++;
                        }
                        count++;
                }
        }
        return 0;
}
