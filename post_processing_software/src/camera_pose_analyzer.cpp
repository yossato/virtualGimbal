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

cv::Mat drawFlowGrid(const cv::Mat flow, const cv::Size grid_size, const cv::Mat src_image)
{
    assert(grid_size <= flow.size());
    assert(flow.type() == CV_32FC1);
    cv::Mat dst_image = src_image.clone();

    

    // Get destinateion / grid_size ratio
    cv::Vec2d ratio = cv::Vec2d((double)dst_image.cols/(double)grid_size.width,(double)dst_image.rows/(double)grid_size.height);
    std::cout << "ratio:" << ratio << std::endl;

    // cv::Mat grid_pos(grid_size,CV_64FC2);
    // for(int r=0;r<grid_size.height;++r)
    // {
    //     for(int c = 0;c<grid_size.width;++c)
    //     {
    //         grid_pos.at<cv::Vec2d>(r,c) = ratio.mul(cv::Vec2d((double)r,(double)c));
    //     }
    // }

    cv::Mat resized_flow;
    cv::resize(flow,resized_flow,grid_size,0.0,0.0,cv::INTER_AREA);
    std::cout << "grid_size:" << grid_size << std::endl;
    // Draw horizontal lines
    for(int y=0;y<grid_size.height;++y)
    {
        for(int x=0;x<grid_size.width-1;++x)
        {
            cv::Point line_begin = (cv::Vec2i)(ratio.mul(cv::Vec2d(x+0.5,y+0.5))+(cv::Vec2d)resized_flow.at<cv::Vec2f>(y,x));
            cv::Point line_end = (cv::Vec2i)(ratio.mul(cv::Vec2d(x+1.5,y+0.5))+(cv::Vec2d)resized_flow.at<cv::Vec2f>(y,x+1));
            cv::line(dst_image,line_begin,line_end,cv::Scalar(0,0,255));
        }
    }

    // Draw vertical lines
    for(int x=0;x<grid_size.width;++x)
    {
        for(int y=0;y<grid_size.height-1;++y)
        {
            cv::Point line_begin = (cv::Vec2i)(ratio.mul(cv::Vec2d(x+0.5,y+0.5))+(cv::Vec2d)resized_flow.at<cv::Vec2f>(y,x));
            cv::Point line_end = (cv::Vec2i)(ratio.mul(cv::Vec2d(x+0.5,y+1.5))+(cv::Vec2d)resized_flow.at<cv::Vec2f>(y+1,x));
            cv::line(dst_image,line_begin,line_end,cv::Scalar(0,0,255));
        }
    }

    return dst_image;
}

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

    cv::namedWindow("frame2", cv::WINDOW_NORMAL);
    cv::namedWindow("source",cv::WINDOW_NORMAL);
    cv::namedWindow("grid_old",cv::WINDOW_NORMAL);
    cv::namedWindow("grid_next",cv::WINDOW_NORMAL);

    // Read video frame
    cv::VideoCapture cap(videoPass);
    cv::UMat umat_old,umat_next;

    float resize_ratio = 1./16.;

    // Read a first frame
    cap >> umat_old;
    cv::resize(umat_old,umat_old,cv::Size(),resize_ratio,resize_ratio,cv::INTER_AREA);
    // Read a next frame
    cap >> umat_next;
    cv::resize(umat_next,umat_next,cv::Size(),resize_ratio,resize_ratio,cv::INTER_AREA);

    cv::UMat mono_old,mono_next;
    cv::UMat float_old,float_next;
    cv::cvtColor(umat_old,mono_old,cv::COLOR_BGRA2GRAY);
    mono_old.convertTo(float_old,CV_32F);
    
    while(!umat_next.empty())
    {

        cv::imshow("source",umat_old);
        
        cv::cvtColor(umat_next,mono_next,cv::COLOR_BGRA2GRAY);
        mono_next.convertTo(float_next,CV_32F);
 
        cv::Mat flow;
        {
            flow = cv::Mat(float_old.size(), CV_32FC2);
            cv::calcOpticalFlowFarneback(float_old, float_next, flow, 0.5, 5, 15, 3, 5, 1.2, 0);
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

        // Draw contour on umat_old and umat_next
        cv::Size grid_size(umat_old.size()/24);
        cv::Mat grid_old = drawFlowGrid(cv::Mat::zeros(flow.size(),CV_32FC2),   grid_size,  umat_old.getMat(cv::ACCESS_READ).clone());
        cv::Mat grid_next = drawFlowGrid(flow,                                  grid_size,  umat_next.getMat(cv::ACCESS_READ).clone());
        cv::imshow("grid_old",grid_old);
        cv::imshow("grid_next",grid_next);

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
        umat_old = std::move(umat_next);
        mono_old = std::move(mono_next);
        float_old = std::move(float_next);
        cap >> umat_next;
        if(umat_next.empty()) break;
        cv::resize(umat_next,umat_next,cv::Size(),resize_ratio,resize_ratio,cv::INTER_AREA);
    }
    return EXIT_SUCCESS;
}