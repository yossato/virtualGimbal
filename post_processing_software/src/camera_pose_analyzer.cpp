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

cv::Mat getGridVertex(cv::Size grid_size, cv::Mat flow)
{

    cv::Mat resized_flow;
    cv::resize(flow,resized_flow,grid_size,0.0,0.0,cv::INTER_AREA);
    // std::cout << "grid_size:" << grid_size << std::endl;

    cv::Vec2d ratio = cv::Vec2d((double)flow.cols/(double)resized_flow.cols,(double)flow.rows/(double)resized_flow.rows);
    // std::cout << "ratio:" << ratio << std::endl;

    cv::Mat grid_vertex(resized_flow.size(),CV_32FC2);
    for(int y=0;y<grid_vertex.rows;++y)
    {
        for(int x=0;x<grid_vertex.cols;++x)
        {
            grid_vertex.at<cv::Vec2f>(y,x) = ((cv::Vec2f)ratio).mul(cv::Vec2f(x+0.5,y+0.5))+resized_flow.at<cv::Vec2f>(y,x);
        }
    }
    return grid_vertex;
}

cv::Mat warpImageUsingGrid(const cv::Mat src_grid_vertex, const cv::Mat dst_grid_vertex, const cv::Mat src_image)
{
    cv::Mat dst_image = cv::Mat::zeros(src_image.size(),src_image.type());
    for(int y=0;y<src_grid_vertex.rows-1;++y)
    {
        for(int x=0;x<src_grid_vertex.cols-1;++x)
        {   
            
            // Get perspective transform matrix
            std::vector<cv::Point2f> src_vertex,dst_vertex;
            src_vertex.push_back(src_grid_vertex.at<cv::Vec2f>(y,x));
            src_vertex.push_back(src_grid_vertex.at<cv::Vec2f>(y,x+1));
            src_vertex.push_back(src_grid_vertex.at<cv::Vec2f>(y+1,x+1));
            src_vertex.push_back(src_grid_vertex.at<cv::Vec2f>(y+1,x));
            dst_vertex.push_back(dst_grid_vertex.at<cv::Vec2f>(y,x));
            dst_vertex.push_back(dst_grid_vertex.at<cv::Vec2f>(y,x+1));
            dst_vertex.push_back(dst_grid_vertex.at<cv::Vec2f>(y+1,x+1));
            dst_vertex.push_back(dst_grid_vertex.at<cv::Vec2f>(y+1,x));
            cv::Mat perspective_transform = cv::getPerspectiveTransform(src_vertex,dst_vertex);

            // printf("x:%d y:%d ",x,y);
            // std::cout << perspective_transform << std::endl;
            if(!isfinite(perspective_transform.at<double>(0,0)) || fabs(perspective_transform.at<double>(0,0)) < std::numeric_limits<double>::epsilon())
            {
                continue;
            }
            // Generate mask
            cv::Mat mask = cv::Mat::zeros(src_image.size(),CV_8UC1);
            std::vector<cv::Vec2i> dst_vertex_i;
            for(auto &el:dst_vertex)
            {
                dst_vertex_i.push_back((cv::Vec2i)(cv::Point2i)el);
            }
            cv::fillConvexPoly(mask,dst_vertex_i,cv::Scalar(255));

            // Warp
            cv::Mat transformed;
            cv::warpPerspective(src_image,transformed,perspective_transform,src_image.size(),cv::INTER_LINEAR);

            // Copy

            transformed.copyTo(dst_image,mask);
        }
    }
    return dst_image;

}

cv::Mat createExtrapolateMap(const cv::Mat &flow, int padding_width)
{
    cv::Mat extrapolated_map(flow.size()+cv::Size(padding_width,padding_width)*2,CV_32FC2,cv::Scalar(-1.f, -1.f));
    // Top side
    for(int c=0;c<flow.cols;++c)
    {
        // Get slope vector
        cv::Vec2f d0 = flow.at<cv::Vec2f>(0,c);
        cv::Vec2f d1 = flow.at<cv::Vec2f>(1,c);
        cv::Vec2f d = d0-d1;
        // Extrapolate all top side
        for(int r=-1,e=-padding_width;r>e;--r)
        {
            // Calculate extrapolated map value
            cv::Vec2f val = d0 + d * fabs((float)r) + cv::Vec2f(c,r);
            // if(val[0] < 0.f || val[1] < 0.f) break;
            if(val[1] < 0.f) break;

            extrapolated_map.at<cv::Vec2f>(r+padding_width,c+padding_width) = val;
        }
    }

    // Bottom side
    for(int c=0;c<flow.cols;++c)
    {
        // Get slope vector
        cv::Vec2f d0 = flow.at<cv::Vec2f>(flow.rows-1,c);
        cv::Vec2f d1 = flow.at<cv::Vec2f>(flow.rows-2,c);
        cv::Vec2f d = d0-d1;
        // Extrapolate all top side
        for(int r=flow.rows,e=flow.rows + padding_width;r<e;++r)
        {
            // Calculate extrapolated map value
            cv::Vec2f val = d0 + d * fabs((float)(r-flow.rows+1)) + cv::Vec2f(c,r);
            // if((val[0] > (flow.rows - 1))  || (val[1] > (flow.rows - 1))) break;
            if(val[1] > (flow.rows-1)) break;

            extrapolated_map.at<cv::Vec2f>(r+padding_width,c+padding_width) = val;
        }
    }

    // Left side
    for(int r=0;r<flow.rows;++r)
    {
        // Get slope vector
        cv::Vec2f d0 = flow.at<cv::Vec2f>(r,0);
        cv::Vec2f d1 = flow.at<cv::Vec2f>(r,1);
        cv::Vec2f d = d0-d1;
        // Extrapolate all top side
        for(int c=-1,e=-padding_width;c>e;--c)
        {
            // Calculate extrapolated map value
            cv::Vec2f val = d0 + d * fabs((float)c) + cv::Vec2f(c,r);
            // if(val[0] < 0.f || val[1] < 0.f) break;
            if(val[0] < 0.f) break;

            extrapolated_map.at<cv::Vec2f>(r+padding_width,c+padding_width) = val;
        }
    }

    // Right side
    for(int r=0;r<flow.rows;++r)
    {
        // Get slope vector
        cv::Vec2f d0 = flow.at<cv::Vec2f>(r,0);
        cv::Vec2f d1 = flow.at<cv::Vec2f>(r,1);
        cv::Vec2f d = d0-d1;
        // Extrapolate all top side
        for(int c=flow.cols,e=flow.cols+padding_width;c<e;++c)
        {
            // Calculate extrapolated map value
            cv::Vec2f val = d0 + d * fabs((float)(c-flow.cols+1)) + cv::Vec2f(c,r);
            // if(val[0] < 0.f || val[1] < 0.f) break;
            if(val[0] > (flow.cols-1)) break;

            extrapolated_map.at<cv::Vec2f>(r+padding_width,c+padding_width) = val;
        }
    }

    // std::cout << extrapolated_map << std::endl;
    return extrapolated_map;
}

cv::Mat drawFlowGrid(const cv::Mat flow, const cv::Size grid_size, const cv::Mat src_image)
{
    // assert(grid_size <= flow.size());
    assert(flow.type() == CV_32FC2);
    cv::Mat dst_image = src_image.clone();

    cv::Mat vertex = getGridVertex(grid_size,flow);

    // Draw horizontal lines
    for(int y=0;y<grid_size.height;++y)
    {
        for(int x=0;x<grid_size.width-1;++x)
        {
            cv::Point line_begin = (cv::Vec2i)vertex.at<cv::Vec2f>(y,x);
            cv::Point line_end = (cv::Vec2i)vertex.at<cv::Vec2f>(y,x+1);
            cv::line(dst_image,line_begin,line_end,cv::Scalar(0,0,255));
        }
    }

    // Draw vertical lines
    for(int x=0;x<grid_size.width;++x)
    {
        for(int y=0;y<grid_size.height-1;++y)
        {
            cv::Point line_begin = (cv::Vec2i)vertex.at<cv::Vec2f>(y,x);
            cv::Point line_end = (cv::Vec2i)vertex.at<cv::Vec2f>(y+1,x);
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
    cv::namedWindow("warped",cv::WINDOW_NORMAL);
    cv::namedWindow("diff_old",cv::WINDOW_NORMAL);
    cv::namedWindow("diff_next",cv::WINDOW_NORMAL);
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
        cv::Size grid_size(umat_old.size()/64);
        cv::Mat grid_old = drawFlowGrid(cv::Mat::zeros(flow.size(),CV_32FC2),   grid_size,  umat_old.getMat(cv::ACCESS_READ).clone());
        cv::Mat grid_next = drawFlowGrid(flow,                                  grid_size,  umat_next.getMat(cv::ACCESS_READ).clone());
        cv::imshow("grid_old",grid_old);
        cv::imshow("grid_next",grid_next);

        if(1){
            cv::Mat old_grid_vertex = getGridVertex(grid_size,cv::Mat::zeros(flow.size(),CV_32FC2));
            cv::Mat next_grid_vertex = getGridVertex(grid_size,flow);
            cv::Mat warped = warpImageUsingGrid(next_grid_vertex,old_grid_vertex,umat_next.getMat(cv::ACCESS_READ).clone());
            cv::imshow("warped",warped);
            cv::Mat warped_32S,old_32S,next_32S,diff;
            
            // Diff old
            {
                warped.convertTo(warped_32S,CV_32SC3);
                umat_old.getMat(cv::ACCESS_READ).clone().convertTo(old_32S,CV_32SC3);
                diff = (warped_32S - old_32S + 127);
                diff.convertTo(diff,CV_8UC3);
                cv::imshow("diff_old",diff);

            }
            // Diff next
            {
                warped.convertTo(warped_32S,CV_32SC3);
                umat_next.getMat(cv::ACCESS_READ).clone().convertTo(next_32S,CV_32SC3);
                diff = (warped_32S - next_32S + 127);
                diff.convertTo(diff,CV_8UC3);
                cv::imshow("diff_next",diff);

            }
        }

        int extrapolate_width = 10;
        cv::Mat extrapolated_map = createExtrapolateMap(flow, extrapolate_width);
        cv::Mat extrapolated_maps[2];
        cv::split(extrapolated_map,extrapolated_maps);
        cv::Mat extrapolated_image;
        cv::remap(umat_next.getMat(cv::ACCESS_READ).clone(),extrapolated_image,extrapolated_maps[0],extrapolated_maps[1],cv::INTER_CUBIC,cv::BORDER_CONSTANT,cv::Scalar(255,0,0));
        // umat_old.getMat(cv::ACCESS_READ).copyTo(extrapolated_image(cv::Rect(extrapolate_width,extrapolate_width,umat_next.cols,umat_next.rows)));
        cv::imshow("x",extrapolated_maps[0]);
        cv::imshow("y",extrapolated_maps[1]);
        // std::cout << extrapolated_map << std::endl;
        cv::imshow("extrapolation",extrapolated_image);

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