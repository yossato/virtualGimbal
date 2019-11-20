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
#include "camera_information.h"

CameraInformation::CameraInformation():camera_name_(""),lens_name_(""),sd_card_rotation_(Eigen::Quaterniond()),
    width_(0),height_(0),fx_(0.),fy_(0.),cx_(0.),cy_(0.),k1_(0.),k2_(0.),p1_(0.),p2_(0.),line_delay_(0.),
    inverse_k1_(0.),inverse_k2_(0.),inverse_p1_(0.),inverse_p2_(0.)
    {}
CameraInformation::CameraInformation(std::string camera_name, std::string lens_name, Eigen::Quaterniond sd_card_rotation, int32_t width, int32_t height, double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2, double line_delay) :
    camera_name_(camera_name),lens_name_(lens_name),sd_card_rotation_(sd_card_rotation),
    width_(width),height_(height),fx_(fx),fy_(fy),cx_(cx),cy_(cy),k1_(k1),k2_(k2),p1_(p1),p2_(p2),line_delay_(line_delay),
    inverse_k1_(0.),inverse_k2_(0.),inverse_p1_(0.),inverse_p2_(0.)
{

}
