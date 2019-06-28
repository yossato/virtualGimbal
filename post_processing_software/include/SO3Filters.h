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
    double time,
    AngularVelocityPtr angular_velocity,
    std::vector<Eigen::Array2d, Eigen::aligned_allocator<Eigen::Array2d>> &contour,
    double zoom,
    VideoPtr video_param,
    const Eigen::VectorXd &filter_coeffs);
// Eigen::VectorXd getKaiserWindow(uint32_t tap_length, uint32_t alpha, bool swap);

bool hasBlackSpace(double time,
                   double zoom,
                   AngularVelocityPtr angular_velocity,
                   VideoPtr video_param,
                   KaiserWindowFilter &filter);
uint32_t bisectionMethod(double time,
                         double zoom,
                         AngularVelocityPtr angular_velocity,
                         VideoPtr video_param,
                         KaiserWindowFilter &filter,
                         int32_t minimum_filter_strength,
                         int32_t maximum_filter_strength,
                         int max_iteration = 1000, uint32_t eps = 1);
// bool isGoodWarp(std::vector<Eigen::Array2d, Eigen::aligned_allocator<Eigen::Array2d>> &contour);
#endif //__SO3FILTERS__H__