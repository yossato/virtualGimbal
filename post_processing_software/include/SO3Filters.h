#ifndef __SO3FILTERS__H__
#define __SO3FILTERS__H__

#include <stdio.h>
#include <Eigen/Dense>
#include <vector>
#include <rotation_param.h>

// Eigen::MatrixXd getFilterCoefficients
void gradientLimit(Eigen::VectorXd &input, double maximum_gradient_);
bool isGoodWarp(std::vector<Eigen::Array2d, Eigen::aligned_allocator<Eigen::Array2d>> &contour);
std::vector<Eigen::Array2d, Eigen::aligned_allocator<Eigen::Array2d>> getSparseContour(VideoPtr video_info, int n);
void getUndistortUnrollingContour(
    double time,
    RotationQuaternionPtr rotation_quaternion,
    std::vector<Eigen::Array2d, Eigen::aligned_allocator<Eigen::Array2d>> &contour,
    double zoom,
    VideoPtr video_param
    );
bool hasBlackSpace(double time,
                   RotationQuaternionPtr rotation_quaternion,
                   double zoom,
                   VideoPtr video_param);
uint32_t bisectionMethod(int32_t frame, int32_t minimum_filter_strength, int32_t maximum_filter_strength, int max_iteration, uint32_t eps);
bool isGoodWarp(std::vector<Eigen::Array2d, Eigen::aligned_allocator<Eigen::Array2d>> &contour);
Eigen::VectorXd getFilterCoefficients(int32_t minimum_filter_strength, int32_t maximum_filter_strength);
#endif //__SO3FILTERS__H__