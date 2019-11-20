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
#ifndef CAMERAINFORMATION_H
#define CAMERAINFORMATION_H

#include <stdio.h>
#include <iostream>
#include <string>
#include <memory>
#include <Eigen/Dense>
class CameraInformation{
public:
    CameraInformation();
    CameraInformation(std::string camera_name,std::string lens_name,Eigen::Quaterniond sd_card_rotation,int32_t width,int32_t height,
                      double fx,double fy,double cx,double cy,double k1,double k2,double p1,
                      double p2,double line_delay);
virtual ~CameraInformation() = default;

    std::string camera_name_;
    std::string lens_name_;
    Eigen::Quaterniond sd_card_rotation_;
    int32_t width_;
    int32_t height_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double k1_;
    double k2_;
    double p1_;
    double p2_;
    double line_delay_; // Unit is second. 
    double inverse_k1_;
    double inverse_k2_;
    double inverse_p1_;
    double inverse_p2_;
};

using CameraInformationPtr = std::shared_ptr<CameraInformation>;

#endif // CAMERAINFORMATION_H
