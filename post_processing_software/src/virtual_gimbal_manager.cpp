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
#include "virtual_gimbal_manager.h"

VirtualGimbalManager::VirtualGimbalManager()
{
}

void VirtualGimbalManager::setVideoParam(const char *file_name)
{
    cv::VideoCapture *Capture = new cv::VideoCapture(file_name); //動画をオープン
    if (!Capture->isOpened())
    {
        throw "Video not found.";
    }
    video_param.reset(new Video(Capture->get(cv::CAP_PROP_FPS)));
    video_param->video_frames = Capture->get(cv::CAP_PROP_FRAME_COUNT);
    video_param->rolling_shutter_time = 0.0;
}

void VirtualGimbalManager::setMeasuredAngularVelocity(const char *file_name)
{
    measured_angular_velocity.reset(new AngularVelocity(readSamplingRateFromJson(file_name)));
    measured_angular_velocity->data = readAngularVelocityFromJson(file_name);
}

void VirtualGimbalManager::setRotation(const char *file_name, CameraInformation& cameraInfo){
    rotation.reset(new Rotation());
    // std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> angular_velocity;
    // if(file_name){
    //     readAngularVelocityFromJson(angular_velocity,file_name);
    // }else{
    //     throw  "Json file not found.";
    // }

    // angularVelocityCoordinateTransformer(angular_velocity,cameraInfo.sd_card_rotation_);
}