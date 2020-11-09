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
#ifndef __SO3FILTERS__H__
#define __SO3FILTERS__H__

#include <stdio.h>
#include <Eigen/Dense>
#include <vector>
#include <rotation_param.h>
#include <boost/math/special_functions/bessel.hpp>
#include <memory>
// Eigen::MatrixXd getFilterCoefficients

void gradientLimit(Eigen::VectorXd &input, double maximum_gradient_);
bool isGoodWarp(std::vector<Eigen::Array2d, Eigen::aligned_allocator<Eigen::Array2d>> &contour, VideoPtr video_param);
std::vector<Eigen::Array2d, Eigen::aligned_allocator<Eigen::Array2d>> getSparseContour(VideoPtr video_info, int n);
void getUndistortUnrollingContour(
    int frame,
    AngularVelocityPtr angular_velocity,
    std::vector<Eigen::Array2d, Eigen::aligned_allocator<Eigen::Array2d>> &contour,
    std::vector<std::pair<int32_t,double>> sync_table,
    double zoom,
    VideoPtr video_param,
    const Eigen::VectorXd &filter_coeffs);
// Eigen::VectorXd getKaiserWindow(uint32_t tap_length, uint32_t alpha, bool swap);

bool hasBlackSpace(int frame,
                   double zoom,
                   AngularVelocityPtr angular_velocity,
                   VideoPtr video_param,
                   Eigen::VectorXd filter_coefficients,
                   std::vector<std::pair<int32_t,double>> &sync_table);
uint32_t bisectionMethod(int frame,
                         double zoom,
                         AngularVelocityPtr angular_velocity,
                         VideoPtr video_param,
                         FilterPtr filter,
                         std::vector<std::pair<int32_t,double>> &sync_table,
                         int32_t minimum_filter_strength,
                         int32_t maximum_filter_strength,
                         int max_iteration = 1000, uint32_t eps = 1);
// bool isGoodWarp(std::vector<Eigen::Array2d, Eigen::aligned_allocator<Eigen::Array2d>> &contour);
#endif //__SO3FILTERS__H__