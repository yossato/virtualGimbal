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

void VirtualGimbalManager::setVideoParam(const char *file_name, CameraInformationPtr info)
{
    std::shared_ptr<cv::VideoCapture> capture = std::make_shared<cv::VideoCapture>(file_name); //動画をオープン
    if (!capture->isOpened())
    {
        throw "Video not found.";
    }
    video_param.reset(new Video(capture->get(cv::CAP_PROP_FPS)));
    video_param->video_frames = capture->get(cv::CAP_PROP_FRAME_COUNT);
    video_param->rolling_shutter_time = 0.0;
    video_param->camera_info = info;
}

void VirtualGimbalManager::setMeasuredAngularVelocity(const char *file_name)
{
    measured_angular_velocity.reset(new AngularVelocity(readSamplingRateFromJson(file_name)));
    measured_angular_velocity->data = readAngularVelocityFromJson(file_name);
}

void VirtualGimbalManager::setEstimatedAngularVelocity(const char *file_name, CameraInformationPtr info, int32_t maximum_synchronize_frames)
{
    std::shared_ptr<cv::VideoCapture> capture = std::make_shared<cv::VideoCapture>(file_name); //動画をオープン

    Eigen::MatrixXd optical_flow;

    calcShiftFromVideo(capture, maximum_synchronize_frames, optical_flow);

    estimated_angular_velocity.reset(new AngularVelocity(capture->get(cv::CAP_PROP_FPS)));
    estimated_angular_velocity->data.resize(optical_flow.rows(), optical_flow.cols());

    estimated_angular_velocity->data.col(0) =
        optical_flow.col(1).unaryExpr([&](double a) { return estimated_angular_velocity->getFrequency() * atan(a / (-info->fy_)); });
    estimated_angular_velocity->data.col(1) =
        optical_flow.col(0).unaryExpr([&](double a) { return estimated_angular_velocity->getFrequency() * -atan(a / (info->fx_)); });
    estimated_angular_velocity->data.col(2) = -estimated_angular_velocity->getFrequency() * optical_flow.col(2);
}

void VirtualGimbalManager::setEstimatedAngularVelocity(Eigen::MatrixXd &angular_velocity, Eigen::VectorXd &confidence, double frequency)
{
    estimated_angular_velocity = std::make_shared<AngularVelocity>(frequency);
    estimated_angular_velocity->confidence = confidence;
    estimated_angular_velocity->data = angular_velocity;
}

void VirtualGimbalManager::setRotation(const char *file_name, CameraInformation &cameraInfo)
{
    rotation.reset(new Rotation());
    // std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> angular_velocity;
    // if(file_name){
    //     readAngularVelocityFromJson(angular_velocity,file_name);
    // }else{
    //     throw  "Json file not found.";
    // }

    // angularVelocityCoordinateTransformer(angular_velocity,cameraInfo.sd_card_rotation_);
}

Eigen::MatrixXd VirtualGimbalManager::estimate()
{
    Eigen::MatrixXd measured_angular_velocity_resampled = measured_angular_velocity->getResampledData(video_param->getFrequency());
    std::cout << measured_angular_velocity->data.block(measured_angular_velocity->data.rows()-100, 0, 100, 3) << std::endl;
    std::cout << measured_angular_velocity_resampled.block(measured_angular_velocity_resampled.rows()-100, 0, 100, 3) << std::endl;
    int32_t diff = measured_angular_velocity_resampled.rows() - estimated_angular_velocity->data.rows();
    std::cout << diff << std::endl;
    std::vector<double> correlation_coefficients(diff + 1);
    for (int32_t frame = 0, end = correlation_coefficients.size(); frame < end; ++frame)
    {
        int32_t number_of_data = estimated_angular_velocity->confidence.cast<int>().array().sum();
        if (0 == number_of_data)
        {
            correlation_coefficients[frame] = std::numeric_limits<double>::max();
        }
        else
        {
            correlation_coefficients[frame] = ((measured_angular_velocity_resampled.block(frame, 0, estimated_angular_velocity->data.rows(), estimated_angular_velocity->data.cols()) - estimated_angular_velocity->data).array().colwise() * estimated_angular_velocity->confidence.array()).abs().sum() / (double)number_of_data;
        }

        if (frame % 100 == 0)
        {
            printf("\r%d / %d", frame, diff);
            std::cout << std::flush;
        }
    }
    // std::cout << correlation_coefficients.block(0,0,100,1) << std::endl;
    int32_t minimum_correlation_frame = std::distance(correlation_coefficients.begin(), min_element(correlation_coefficients.begin(), correlation_coefficients.end()));
    //最小値サブピクセル推定
    double minimum_correlation_subframe = 0.0;
    if (minimum_correlation_frame == 0)
    { //位置が最初のフレームで一致している場合
        minimum_correlation_subframe = 0.0;
    }
    else if (minimum_correlation_frame == (diff - 1))
    { //末尾
        minimum_correlation_subframe = (double)(diff - 1);
    }
    else
    { //その他
        minimum_correlation_subframe = -(correlation_coefficients[minimum_correlation_frame + 1] - correlation_coefficients[minimum_correlation_frame - 1]) / (2 * correlation_coefficients[minimum_correlation_frame - 1] - 4 * correlation_coefficients[minimum_correlation_frame] + 2 * correlation_coefficients[minimum_correlation_frame + 1]);
    }
    minimum_correlation_subframe += (double)minimum_correlation_frame;
    std::cout << std::endl << minimum_correlation_subframe << std::endl;

    Eigen::MatrixXd retval(correlation_coefficients.size(),1);
    for(int i=0,e=correlation_coefficients.size();i<e;++i){
        retval(i,0) = correlation_coefficients[i];
    }
    return retval;
}