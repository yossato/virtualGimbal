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
// #include <opencv2/opencv.hpp>
// #include <opencv2/video/tracking.hpp>
// #include <iostream>
// #include <cassert>
// #include <cmath>
// #include <Eigen/Dense>
// #include <memory>
#include "calcShift.hpp"

PointPairs getFeaturePointsPairsFromVideo(const char *filename, int total_frames)
{
    PointPairs point_pairs;

    // Open Video
    assert(0 != total_frames);
    cv::VideoCapture cap(filename);
    assert(cap.isOpened());

    assert(cap.get(cv::CAP_PROP_FRAME_COUNT) >= total_frames);

    cv::Mat cur, cur_grey;
    cv::Mat prev, prev_grey;

    // Get first frame
    cap >> prev;

    // Trimming and make it mono
    int width  = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    if((width >= 640) && (height >= 480))
    {
        cv::cvtColor(prev(cv::Rect((prev.cols-640)/2,(prev.rows-480)/2,640,480)), prev_grey, cv::COLOR_BGR2GRAY);
    }else
    {
        cv::cvtColor(prev, prev_grey, cv::COLOR_BGR2GRAY);
    }
    
    // Step 1 - Get previous to current frame transformation (dx, dy, da) for all frames
    std::vector <cv::Vec3d> prev_to_cur_transform; // previous to current
    // int k=1;


    
    for(int frame=0;frame<total_frames;++frame) {
        cap >> cur;

        if(cur.data == NULL) {
            break;
        }

        // Trimming
        if((width >= 640) && (height >= 480))
        {
            cv::cvtColor(cur(cv::Rect((cur.cols-640)/2,(cur.rows-480)/2,640,480)), cur_grey, cv::COLOR_BGR2GRAY);
        }else
        {
            cv::cvtColor(cur, cur_grey, cv::COLOR_BGR2GRAY);
        }


        // vector from prev to cur
        std::vector <cv::Point2f> prev_corner, cur_corner;
        point_pairs.push_back(std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>>());
        std::vector <cv::Point2f> &prev_corner2 = point_pairs.back().first;
        std::vector <cv::Point2f> &cur_corner2 = point_pairs.back().second;
        std::vector <uchar> status;
        std::vector <float> err;

        cv::goodFeaturesToTrack(prev_grey, prev_corner, 200, 0.01, 30);
        cv::calcOpticalFlowPyrLK(prev_grey, cur_grey, prev_corner, cur_corner, status, err);

        // weed out bad matches
        for(size_t i=0; i < status.size(); i++) {
            if(status[i]) {
                prev_corner2.push_back(prev_corner[i]);
                cur_corner2.push_back(cur_corner[i]);
            }
        }

        

        cur.copyTo(prev);
        cur_grey.copyTo(prev_grey);

        printf("Frame: %d/%d - good optical flow: %lu       \r",frame,total_frames,prev_corner2.size());
    }
    
    return point_pairs;
}

void convertFeaturePointsPairsToImageTranslationAndRotation(const PointPairs &point_pairs, Eigen::MatrixXd &translation_and_rotation, Eigen::MatrixXd &confidence)
{
    size_t total_frames = point_pairs.size();
    translation_and_rotation = Eigen::MatrixXd::Zero(total_frames,3);
    confidence = Eigen::MatrixXd::Zero(total_frames,1);

    for(size_t frame=0;frame<point_pairs.size();++frame) {
        
        const std::vector<cv::Point2f> &prev_corner2 = point_pairs[frame].first;
        const std::vector<cv::Point2f> &cur_corner2 = point_pairs[frame].second;
        
        // translation + rotation only
        try{
            cv::Mat T = cv::estimateAffinePartial2D(prev_corner2, cur_corner2); 

            // in rare cases no transform is found. We'll just use the last known good transform.
            if(T.data == NULL) {
                translation_and_rotation.row(frame) << 0.0, 0.0, 0.0;
                confidence.row(frame) << 0.0;
            }else{
                double dx = T.at<double>(0,2);
                double dy = T.at<double>(1,2);
                double da = atan2(T.at<double>(1,0), T.at<double>(0,0));
                translation_and_rotation.row(frame) << dx, dy, da;
                confidence.row(frame) << 1.0;
            }
        }catch(...){
            translation_and_rotation.row(frame) << 0.0, 0.0, 0.0;
            confidence.row(frame) << 0.0;
        }
    }
}

void CalcShiftFromVideo(const char *filename, int total_frames, Eigen::MatrixXd &optical_flow, Eigen::MatrixXd &confidence){
    // Open Video
    assert(0 != total_frames);
    cv::VideoCapture cap(filename);
    assert(cap.isOpened());

    assert(cap.get(cv::CAP_PROP_FRAME_COUNT) >= total_frames);
    optical_flow = Eigen::MatrixXd::Zero(total_frames,3);
    confidence = Eigen::MatrixXd::Zero(total_frames,1);

    cv::Mat cur, cur_grey;
    cv::Mat prev, prev_grey;

    // Get first frame
    cap >> prev;

    // Trimming and make it mono
    int width  = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    if((width >= 640) && (height >= 480))
    {
        cv::cvtColor(prev(cv::Rect((prev.cols-640)/2,(prev.rows-480)/2,640,480)), prev_grey, cv::COLOR_BGR2GRAY);
    }else
    {
        cv::cvtColor(prev, prev_grey, cv::COLOR_BGR2GRAY);
    }
    
    // Step 1 - Get previous to current frame transformation (dx, dy, da) for all frames
    std::vector <cv::Vec3d> prev_to_cur_transform; // previous to current
    // int k=1;


    
    for(int frame=0;frame<total_frames;++frame) {
        cap >> cur;

        if(cur.data == NULL) {
            break;
        }

        // Trimming
        if((width >= 640) && (height >= 480))
        {
            cv::cvtColor(cur(cv::Rect((cur.cols-640)/2,(cur.rows-480)/2,640,480)), cur_grey, cv::COLOR_BGR2GRAY);
        }else
        {
            cv::cvtColor(cur, cur_grey, cv::COLOR_BGR2GRAY);
        }


        // vector from prev to cur
        std::vector <cv::Point2f> prev_corner, cur_corner;
        std::vector <cv::Point2f> prev_corner2, cur_corner2;
        std::vector <uchar> status;
        std::vector <float> err;

        cv::goodFeaturesToTrack(prev_grey, prev_corner, 200, 0.01, 30);
        cv::calcOpticalFlowPyrLK(prev_grey, cur_grey, prev_corner, cur_corner, status, err);

        // weed out bad matches
        for(size_t i=0; i < status.size(); i++) {
            if(status[i]) {
                prev_corner2.push_back(prev_corner[i]);
                cur_corner2.push_back(cur_corner[i]);
            }
        }

        // translation + rotation only
        try{
            cv::Mat T = cv::estimateAffinePartial2D(prev_corner2, cur_corner2); 

            // in rare cases no transform is found. We'll just use the last known good transform.
            if(T.data == NULL) {
                optical_flow.row(frame) << 0.0, 0.0, 0.0;
                confidence.row(frame) << 0.0;
            }else{
                double dx = T.at<double>(0,2);
                double dy = T.at<double>(1,2);
                double da = atan2(T.at<double>(1,0), T.at<double>(0,0));
                optical_flow.row(frame) << dx, dy, da;
                confidence.row(frame) << 1.0;
            }
        }catch(...){
            optical_flow.row(frame) << 0.0, 0.0, 0.0;
            confidence.row(frame) << 0.0;
        }

        cur.copyTo(prev);
        cur_grey.copyTo(prev_grey);

        printf("Frame: %d/%d - good optical flow: %lu       \r",frame,total_frames,prev_corner2.size());
    }
    // return prev_to_cur_transform;
    return;
}
