
#include <stdio.h>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "json_tools.hpp"
#include "calcShift.hpp"

int getVideoLength(const char *videoName)
{
    std::shared_ptr<cv::VideoCapture> Capture = std::make_shared<cv::VideoCapture>(videoName); //動画をオープン
    assert(Capture->isOpened());
    return (int)Capture->get(cv::CAP_PROP_FRAME_COUNT);
}

int main(int argc, char **argv)
{
    if (1 == argc)
    {
        printf("VirtualGimbal angular velocity estimator\r\n"
               "Run with video file path.\r\n");
        return 1;
    }

    Eigen::MatrixXd optical_flow, confidence;
    //動画からオプティカルフローを計算する
    for (int i = 1; i < argc; ++i)
    {
        printf("Processing... %s  \r\n", argv[i]);
        //ファイルが存在する？
        //動画ファイル？
        cv::VideoCapture video(argv[i]);
        if (!video.isOpened())
        {
            std::cerr << argv[i] << " can't be opened." << std::endl;
            continue;
        }
       
        // Jsonがすでにある？
        if (!jsonExists(std::string(argv[i])))
        {
            CalcShiftFromVideo(argv[i], getVideoLength(argv[i]), optical_flow, confidence); //ビデオからオプティカルフローを用いてシフト量を算出
            writeOpticalFrowToJson(std::string(argv[i]),optical_flow,confidence);
            std::cout << argv[i] << " done." << std::endl;
        }else{
            std::cout << argv[i] << " already exists." << std::endl;
        }
    }
    return 0;
}