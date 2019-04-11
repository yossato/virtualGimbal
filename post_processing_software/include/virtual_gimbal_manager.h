/**
* This file is part of VirtualGimbal.
*
* Copyright 2019 Yoshiaki Sato <virtualgimbal at xa2 dot so-net dot ne dot jp>
*
* VirtualGimbal is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.

* VirtualGimbal is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.

* You should have received a copy of the GNU General Public License
* along with VirtualGimbal.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef __VIRTUAL_GIMBAL_MANAGER_H__
#define __VIRTUAL_GIMBAL_MANAGER_H__

#include <stdio.h>
#include <memory>
#include "rotation_param.h"
#include <opencv2/opencv.hpp>
#include "json_tools.hpp"
#include "camera_information.h"
#include "calcShift.hpp"

class VirtualGimbalManager
{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VirtualGimbalManager();
    void setVideoParam(const char* file_name,  CameraInformationPtr info);
    void setMeasuredAngularVelocity(const char* file_name);
    void setEstimatedAngularVelocity(const char* file_name, CameraInformationPtr info, int32_t maximum_synchronize_frames=1000);
    void setEstimatedAngularVelocity(Eigen::MatrixXd &angular_velocity, Eigen::VectorXd &confidence, double frequency);
    void setRotation(const char *file_name, CameraInformation& cameraInfo);
    Eigen::MatrixXd estimate();
protected:
    RotationPtr rotation;
    AngularVelocityPtr measured_angular_velocity;
    AngularVelocityPtr estimated_angular_velocity;
    VideoPtr video_param;
    FilterPtr filter;

    template <typename _Tp, typename _Alloc = std::allocator<_Tp>>
    void angularVelocityCoordinateTransformer(std::vector<_Tp, _Alloc> &angular_velocity, const Eigen::Quaterniond &rotation)
    {
        Eigen::Quaterniond avq;
        avq.w() = 0.0;
        for (auto &el : angular_velocity)
        {
            avq.x() = el[0];
            avq.y() = el[1];
            avq.z() = el[2];
            avq = rotation * avq * rotation.conjugate(); //Rotate angular velocity vector
            el[0] = avq.x();
            el[1] = avq.y();
            el[2] = avq.z();
        }
    }
};

#endif // __VIRTUAL_GIMBAL_MANAGER_H__
