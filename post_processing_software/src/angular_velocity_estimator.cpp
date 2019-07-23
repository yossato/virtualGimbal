
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
        printf("%d %s\r\n", i, argv[i]);
        //ファイルが存在する？
        //動画ファイル？
        cv::VideoCapture video(argv[i]);
        if (!video.isOpened())
        {
            std::cerr << "Warning: Video file " << argv[i] << " is not found. It's skipped." << std::endl;
            continue;
        }
       
        // Jsonがすでにある？
        if (!jsonExists(std::string(argv[i])))
        {
            CalcShiftFromVideo(argv[i], getVideoLength(argv[i]), optical_flow, confidence); //ビデオからオプティカルフローを用いてシフト量を算出
            writeOpticalFrowToJson(optical_flow, std::string(argv[i]));
        }
    }
    return 0;
}