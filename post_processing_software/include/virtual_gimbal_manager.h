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
#include "Eigen/Dense"
#include "rotation_math.h"
class VirtualGimbalManager
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VirtualGimbalManager();
  void setVideoParam(const char *file_name, CameraInformationPtr info);
  static std::string getVideoSize(const char *videoName);
  void setMeasuredAngularVelocity(const char *file_name, CameraInformationPtr info = nullptr);
  void setEstimatedAngularVelocity(const char *file_name, CameraInformationPtr info, int32_t maximum_synchronize_frames = 1000);
  void setEstimatedAngularVelocity(Eigen::MatrixXd &angular_velocity, Eigen::VectorXd confidence, double frequency=0.0);
  void setRotation(const char *file_name, CameraInformation &cameraInfo);
  // void getEstimatedAndMeasuredAngularVelocity(Eigen::MatrixXd &data);
  Eigen::MatrixXd getCorrelationCoefficient();
  Eigen::MatrixXd getSynchronizedMeasuredAngularVelocity();
  std::map<int, std::vector<cv::Point2d>> getCornerDictionary(cv::Size &pattern_size, bool debug_speedup = false, bool Verbose = false);
  Eigen::MatrixXd estimateAngularVelocity(const std::map<int, std::vector<cv::Point2d>> &corner_dict, const std::vector<cv::Point3d> &world_points, Eigen::VectorXd &confidence);
  void estimateAngularVelocity(Eigen::MatrixXd &estimated_angular_velocity, Eigen::MatrixXd &confidence,int frames=1000);
  Eigen::MatrixXd getRotationQuaternions();
  void getUndistortUnrollingChessBoardPoints(double time_offset, const std::pair<int, std::vector<cv::Point2d>> &corner_dict, std::vector<cv::Point2d> &dst, double line_delay=0.0);
  void getUndistortUnrollingChessBoardPoints(double time, const std::vector<cv::Point2d> &src, std::vector<cv::Point2d> &dst, double line_delay = 0.0);
  double computeReprojectionErrors(const std::vector<std::vector<cv::Point3d>> &objectPoints,
                                   const std::vector<std::vector<cv::Point2d>> &imagePoints,
                                   const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
                                   const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                   std::vector<double> &residuals, bool fisheye = false);
  void spin();
  
protected:
  RotationPtr rotation;
  AngularVelocityPtr measured_angular_velocity;
  AngularVelocityPtr estimated_angular_velocity;
  RotationQuaternionPtr rotation_quaternion;
  //Synchronize
  AngularVelocityPtr resampled_synchronized_angular_velocity;
  ResamplerParameterPtr resampler_parameter_;

  VideoPtr video_param;
  FilterPtr filter;

  void rotateAngularVelocity(Eigen::MatrixXd &angular_velocity, const Eigen::Quaterniond &rotation)
  {
    Eigen::Quaterniond avq;
    avq.w() = 0.0;
    for (int32_t row = 0, e = angular_velocity.rows(); row < e; ++row)
    {
      avq.x() = angular_velocity(row, 0);
      avq.y() = angular_velocity(row, 1);
      avq.z() = angular_velocity(row, 2);
      avq = rotation * avq * rotation.conjugate(); //Rotate angular velocity vector
      angular_velocity(row, 0) = avq.x();
      angular_velocity(row, 1) = avq.y();
      angular_velocity(row, 2) = avq.z();
    }
  }
};

#endif // __VIRTUAL_GIMBAL_MANAGER_H__
