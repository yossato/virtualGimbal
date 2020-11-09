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
#include "levenbergMarquardt.hpp"
#include "camera_information.h"

//void calcDistortCoeff(const cv::Mat &matIntrinsic, const cv::Mat &matDistort, const cv::Size &imageSize, cv::Mat &matInvDistort){
void calcInverseDistortCoeff(CameraInformation &camera_info){
    //逆歪パラメータを求める
    std::vector<double> refPointsX;
    std::vector<double> refPointsY;
    Matrix3d intrinsic;
    intrinsic << 	camera_info.fx_, 0., camera_info.cx_,
            0., camera_info.fy_, camera_info.cy_,
            0., 0., 1.;

//    std::cout << intrinsic << std::endl;
    VectorXd distortionCoeff(4);
    distortionCoeff << camera_info.k1_,camera_info.k2_,camera_info.p1_,camera_info.p2_;
//    std::cout << distortionCoeff << std::endl;
    int step = 20;
    for(int v=0;v<=camera_info.height_;v+=step){
        for(int u=0;u<=camera_info.width_;u+=step){
            refPointsX.push_back((double)u);
            refPointsY.push_back((double)v);
        }
    }
    //歪補正
    std::vector<double> undistortedPointsX;
    std::vector<double> undistortedPointsY;
    double fx = camera_info.fx_;
    double fy = camera_info.fy_;
    double cx = camera_info.cx_;
    double cy = camera_info.cy_;
    double k1 = camera_info.k1_;
    double k2 = camera_info.k2_;
    double p1 = camera_info.p1_;
    double p2 = camera_info.p2_;
    for(int i=0,e=refPointsX.size();i<e;i++){
        double u = refPointsX[i];
        double v = refPointsY[i];

        double x1 = (u - cx)/fx;
        double y1 = (v - cy)/fy;

        double r = sqrt(x1*x1+y1*y1);

        double x2 = x1*(1.0+k1*r*r+k2*r*r*r*r)+2.0*p1*x1*y1+p2*(r*r+2.0*x1*x1);
        double y2 = y1*(1.0+k1*r*r+k2*r*r*r*r)+p1*(r*r+2.0*y1*y1)+2.0*p2*x1*y1;
        double mapx = x2*fx+cx;
        double mapy = y2*fy+cy;
        undistortedPointsX.push_back(mapx);
        undistortedPointsY.push_back(mapy);
    }
    //最適化
    printf("Before:\t%f,%f,%f,%f\r\n",k1,k2,p1,p2);
    calc_invert_distortion_coeff functor2(distortionCoeff.size(),refPointsX.size(), undistortedPointsX, undistortedPointsY,
                                          refPointsX, refPointsY, intrinsic);

    NumericalDiff<calc_invert_distortion_coeff> numDiff2(functor2);
    LevenbergMarquardt<NumericalDiff<calc_invert_distortion_coeff> > lm2(numDiff2);
    /* int info = */lm2.minimize(distortionCoeff);
    printf("After:\t%f,%f,%f,%f\r\n",distortionCoeff[0],distortionCoeff[1],distortionCoeff[2],distortionCoeff[3]);

//    matInvDistort = (cv::Mat_<double>(1, 4) << distortionCoeff[0],distortionCoeff[1],distortionCoeff[2],distortionCoeff[3]);
    camera_info.inverse_k1_ = distortionCoeff[0];
    camera_info.inverse_k2_ = distortionCoeff[1];
    camera_info.inverse_p1_ = distortionCoeff[2];
    camera_info.inverse_p2_ = distortionCoeff[3];
}
