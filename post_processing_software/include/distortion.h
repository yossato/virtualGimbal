#ifndef DISTORTION_H
#define DISTORTION_H

#include <stdio.h>
#include "levenbergMarquardt.hpp"
#include "camera_information.h"

void calcInverseDistortCoeff(CameraInformation &camera_info);

#endif // DISTORTION_H
