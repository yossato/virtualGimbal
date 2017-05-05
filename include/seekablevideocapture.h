#ifndef SEEKABLEVIDEOCAPTURE_H
#define SEEKABLEVIDEOCAPTURE_H

#include <stdio.h>
#include <stdint.h>
//OpenCV
#include <opencv2/opencv.hpp>
#include <deque>

class seekableVideoCapture
{
public:
    seekableVideoCapture(char *videoFileName, int32_t buffLength);
    bool getFrame(int32_t position,cv::Mat &frame);
    bool getFrameForMIP(int32_t position,cv::Mat &frame);
//    seekableVideoCapture();
private:
    cv::VideoCapture capture;
    std::deque<cv::Mat> buff;
    //int32_t framePosition; //dequeのfrontのフレーム位置。最初は0だが、pop_frontするたびに1増やす。
    int32_t buffLen;
};

#endif // SEEKABLEVIDEOCAPTURE_H
