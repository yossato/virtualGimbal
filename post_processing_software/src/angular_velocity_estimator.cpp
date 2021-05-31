/*************************************************************************
*  Software License Agreement (BSD 3-Clause License)
*  
*  Copyright (c) 2019, Yoshiaki Sato
*  All rights reserved.
*  
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*  
*  1. Redistributions of source code must retain the above copyright notice, this
*     list of conditions and the following disclaimer.
*  
*  2. Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*  
*  3. Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived from
*     this software without specific prior written permission.
*  
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*************************************************************************/
#include <stdio.h>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "json_tools.hpp"
#include "calc_shift.hpp"

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
            calcShiftFromVideo(argv[i], getVideoLength(argv[i]), optical_flow, confidence); //ビデオからオプティカルフローを用いてシフト量を算出
            writeOpticalFrowToJson(std::string(argv[i]),optical_flow,confidence);
            std::cout << argv[i] << " done." << std::endl;
        }else{
            std::cout << argv[i] << " already exists." << std::endl;
        }
    }
    return 0;
}