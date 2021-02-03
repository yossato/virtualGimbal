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
#include <unistd.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <inpainting.hpp>
int main(int argc, char **argv)
{

    // Check arguments
    char *videoPass = NULL;
    int opt;
    int map_col = 5;
    int map_row = 5; 
    int window_height = 100;
    int window_width = 100;
    float colored_map_gain = 2.0;
    while ((opt = getopt(argc, argv, "i:c::r::w::g::h::")) != -1)
    {
        switch (opt)
        {
        case 'i': //input video file pass
            videoPass = optarg;
            printf("videoPass %s\r\n",videoPass);
            break;
        case 'c':   // Map size
            map_col = std::stoi(optarg);
            printf("map col %d\r\n",map_col);
            assert(map_col > 0);
            break;
        case 'r':   // Map size
            map_row = std::stoi(optarg);
            printf("map row %d\r\n",map_row);
            assert(map_row > 0);
            break;
        case 'w':   // window width
            window_width = std::stoi(optarg);
            printf("window_width %d\r\n",window_width);
            assert(window_width > 0);
            break;
        case 'g':   // colored_map_gain
            colored_map_gain = std::stof(optarg);
            printf("colored_map_gain %f\r\n",colored_map_gain);
            assert(colored_map_gain > 0);
            break;
        case 'h':   // window height
            window_height = std::stoi(optarg);
            printf("window_height %d\r\n",window_height);
            assert(window_height > 0);
            break;

        default:
            return EXIT_FAILURE;
        }
    }

    // cv::namedWindow("inpaint",cv::WINDOW_NORMAL);
    // cv::namedWindow("colored_map",cv::WINDOW_NORMAL);
    // cv::namedWindow("colored_map_0",cv::WINDOW_NORMAL);
    // cv::namedWindow("colored_map_1",cv::WINDOW_NORMAL);
    // cv::namedWindow("colored_map_2",cv::WINDOW_NORMAL);

    cv::namedWindow("frame2", cv::WINDOW_NORMAL);
    cv::namedWindow("source",cv::WINDOW_NORMAL);

    // Read video frame
    cv::VideoCapture cap(videoPass);
    cv::UMat umat_src,umat_next;

    // Read a first frame
    cap >> umat_src;
    // Read a next frame
    cap >> umat_next;
    while(!umat_next.empty())
    {

        cv::UMat mini_color;
        cv::resize(umat_src,mini_color,cv::Size(),1./16.,1./16.,cv::INTER_AREA);
        cv::imshow("source",mini_color);

        cv::UMat mono_latest;
        cv::UMat mono_next;
        cv::cvtColor(umat_src,mono_latest,cv::COLOR_BGRA2GRAY);
        cv::cvtColor(umat_next,mono_next,cv::COLOR_BGRA2GRAY);
        cv::Mat float_latest,float_next;
        mono_latest.convertTo(float_latest,CV_32F);
        mono_next.convertTo(float_next,CV_32F);
        // cv::Size map_size = cv::Size(map_col,map_row);
        // cv::Size window_size = cv::Size(window_width,window_height);
        // cv::Mat map = calculateOpticalFlow(map_size,window_size,float_latest,float_next);
        
        // cv::Mat visualize_image = umat_src.getMat(cv::ACCESS_RW).clone();
        // visualizeInpaintingMap(visualize_image,window_size,map);
        // cv::resize(visualize_image,visualize_image,cv::Size(),0.3,0.3);
        // cv::imshow("inpaint",visualize_image);

        // cv::Mat colored_map = generateOpticalFlowMap(map,colored_map_gain);
        // cv::imshow("colored_map",colored_map);

        // cv::Mat pyramid_float_latest = float_latest.clone();
        // cv::Mat pyramid_float_next = float_next.clone();
        // cv::Size pyramid_map_size = map_size;
        // std::vector<cv::Mat> pyramid_colored_map;
        // double pyramid_colored_map_gain = colored_map_gain;
        // for(int i=0;i<5;++i)
        // {
        //     cv::resize(pyramid_float_latest,pyramid_float_latest,pyramid_float_latest.size()/2,0,0,cv::INTER_AREA);
        //     cv::resize(pyramid_float_next,pyramid_float_next,pyramid_float_next.size()/2,0,0,cv::INTER_AREA);
        //     pyramid_map_size = pyramid_map_size/2;
        //     pyramid_colored_map_gain *= 2.;
        //     if(pyramid_float_latest.cols < window_size.width) break;
        //     if(pyramid_float_latest.rows < window_size.height) break;

        //     cv::Mat pyramid_map = calculateOpticalFlow(pyramid_map_size,window_size,pyramid_float_latest,pyramid_float_next);
        //     cv::Mat pyramid_colored_map = generateOpticalFlowMap(pyramid_map,pyramid_colored_map_gain);
        //     cv::imshow(std::string("colored_map_")+std::to_string(i),pyramid_colored_map); 
        // }

        {
            cv::Mat flow(float_latest.size(), CV_32FC2);
            cv::Mat mini_float_latest,mini_float_next;
            cv::resize(float_latest,mini_float_latest,cv::Size(),1./16.,1./16.,cv::INTER_AREA);
            cv::resize(float_next,mini_float_next,cv::Size(),1./16.,1./16.,cv::INTER_AREA);
            cv::calcOpticalFlowFarneback(mini_float_latest, mini_float_next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
            // visualization
            cv::Mat flow_parts[2];
            cv::split(flow, flow_parts);
            cv::Mat magnitude, angle, magn_norm;
            cv::cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
            cv::normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
            angle *= ((1.f / 360.f) * (180.f / 255.f));
            //build hsv image
            cv::Mat _hsv[3], hsv, hsv8, bgr;
            _hsv[0] = angle;
            _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
            _hsv[2] = magn_norm;
            cv::merge(_hsv, 3, hsv);
            hsv.convertTo(hsv8, CV_8U, 255.0);
            cv::cvtColor(hsv8, bgr, cv::COLOR_HSV2BGR);
            cv::imshow("frame2", bgr);
        }

        /*{
            // build pyramids
            std::vector<cv::Mat> vec_pyramid_latest,vec_pyramid_next;
            vec_pyramid_latest.push_back(float_latest.clone());
            vec_pyramid_next.push_back(float_next.clone());
            for(int i=0;i<5;++i)
            {
                cv::Mat lastest = vec_pyramid_latest.back();
                cv::Mat next = vec_pyramid_next.back();

                if(lastest.size().width % 2
                || lastest.size().height % 2
                || lastest.size().width < window_size.width
                || lastest.size().height < window_size.height)
                {
                    break;
                }

                vec_pyramid_next.push_back(cv::Mat());
                vec_pyramid_next.push_back(cv::Mat());
                cv::resize(lastest,vec_pyramid_latest.back(),cv::Size(),0.5,0.5,cv::INTER_AREA);
                cv::resize(next,vec_pyramid_next.back(),cv::Size(),0.5,0.5,cv::INTER_AREA);
                
            }
            // Calculate optical flow from the pyramids
            for(int i=vec_pyramid_latest.size()-1;i>=0;++i)
            {
                cv::Mat optical_flow = calculateOpticalFlow(pyramid_map_size,window_size,pyramid_float_latest,pyramid_float_next);
            }

        }*/

        uint8_t key = (uint8_t)cv::waitKey(1);
        if(key == 'q')
        {
            break;
        }
        else if ('s' == key)
        {
            sleep(1);
            key = cv::waitKey(0);
            if ('q' == key)
            {
                cv::destroyAllWindows();
                break;
            }
        }
        umat_src = std::move(umat_next);
        cap >> umat_next;
    }
    return EXIT_SUCCESS;
}