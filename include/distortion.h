#ifndef DISTORTION_H
#define DISTORTION_H

#include <stdio.h>
#include "levenbergMarquardt.hpp"

void calcDistortCoeff(const cv::Mat &matIntrinsic, const cv::Mat &matDistort, const cv::Size &imageSize, cv::Mat &matInvDistort);

#endif // DISTORTION_H
