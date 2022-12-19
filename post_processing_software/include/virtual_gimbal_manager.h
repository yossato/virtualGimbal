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
#include "SO3Filters.h"
#include "cl_manager.h"
#include "multi_thread_video_writer.h"
#include <chrono>         // std::chrono::seconds
#include <map>
#include <Eigen/Dense>
#include "inpainting.hpp"
#include "data_collection.h"
using PointPair = std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f> >;
using PointPairs = std::vector<std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f> >>;
using SyncTable = std::vector<std::pair<int32_t,double>>;
using MeasuredFrame = double;
using EstimatedFrame = int32_t;
class VirtualGimbalManager
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VirtualGimbalManager();
  VirtualGimbalManager(size_t queue_size);
  void setVideoParam(const char *file_name, CameraInformationPtr info);
  static std::string getVideoSize(const char *videoName);
  void setMeasuredAngularVelocity(const char *file_name, CameraInformationPtr info = nullptr);
  void setEstimatedAngularVelocity(const char *file_name, CameraInformationPtr info, int32_t maximum_synchronize_frames = 1000);
  void setEstimatedAngularVelocity(Eigen::MatrixXd &angular_velocity, Eigen::VectorXd confidence, double frequency=0.0);
  void setRotation(const char *file_name, CameraInformation &cameraInfo);
  void setFilter(FilterPtr filter);
  // void getEstimatedAndMeasuredAngularVelocity(Eigen::MatrixXd &data);
  Eigen::VectorXd getCorrelationCoefficient(int32_t begin=0, int32_t length=0, double frequency=0.0);
  // Eigen::VectorXd getCorrelationCoefficient2(int32_t center, int32_t length, double frequency=0.0);
  double getSubframeOffsetInSecond(Eigen::VectorXd &correlation_coefficients,int32_t begin=0, int32_t length=0, double frequency=0.0);
  double getSubframeOffset(Eigen::VectorXd &correlation_coefficients,int32_t center, int32_t length, double frequency=0.0);
  double getMeasuredFramePositionFrom(int32_t estimated_frame_position, int32_t length);
  // void setResamplerParameter(double start, double new_frequency = 0.0);
  // void setResamplerParameter(ResamplerParameterPtr param);
  // Eigen::MatrixXd getSynchronizedMeasuredAngularVelocity();
  std::map<int, std::vector<cv::Point2d>> getCornerDictionary(cv::Size &pattern_size, bool debug_speedup = false, bool Verbose = false);
  
  PointPairs getFeaturePointsPairs();
  void estimateAngularVelocity(const PointPairs &point_pairs, Eigen::MatrixXd &estimated_angular_velocity, Eigen::MatrixXd &confidence);
  void getAngularVelocityFromJson(Eigen::MatrixXd &estimated_angular_velocity, Eigen::MatrixXd &confidence);

  // void estimateAngularVelocity(Eigen::MatrixXd &estimated_angular_velocity, Eigen::MatrixXd &confidence);
  // Eigen::MatrixXd getRotationQuaternions();
  // void getUndistortUnrollingChessBoardPoints(double time_offset, const std::pair<int, std::vector<cv::Point2d>> &corner_dict, std::vector<cv::Point2d> &dst, double line_delay=0.0);
  // void getUndistortUnrollingChessBoardPoints(double time, const std::vector<cv::Point2d> &src, std::vector<cv::Point2d> &dst, double line_delay = 0.0);
  double computeReprojectionErrors(const std::vector<std::vector<cv::Point3d>> &objectPoints,
                                   const std::vector<std::vector<cv::Point2d>> &imagePoints,
                                   const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
                                   const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                   std::vector<double> &residuals, bool fisheye = false);
  Eigen::VectorXd getFilterCoefficients(double zoom,
                                      FilterPtr filter,
                                      std::vector<std::pair<int32_t,double>> &sync_table, 
                                      int32_t strongest_filter_param, int32_t weakest_filter_param);
  int spin(double zoom, FilterPtr filter,Eigen::VectorXd &filter_strength, std::vector<std::pair<int32_t,double>> &sync_table, bool show_image = true);
  int spinInpainting(double zoom, std::vector<std::pair<int32_t, double>> &sync_table, FilterPtr filter, size_t buffer_size, int filter_strength=199, bool show_image = true);
  cv::Point2f warp_undistort(const cv::Point2f &p, float zoom_ratio, const std::vector<float> &stabilized_angle_matrices);

  int spinAnalyse(double zoom, FilterPtr filter,Eigen::VectorXd &filter_strength, std::vector<std::pair<int32_t,double>> &sync_table, const PointPairs &point_pair,char *experimental_param_json_path);
  bool fillPixelValues(cv::ocl::Context &context, double zoom, std::vector<float> stabilized_angle_matrices, uint8_t distance, UMatPtr &source_image, UMatPtr &dest_image);
  bool interpolatePixels(cv::ocl::Context &context, UMatPtr &past, UMatPtr &future, UMatPtr &output);
  void setMaximumGradient(double value);
  void enableWriter(const char *video_path);
  const char *kernel_name = "stabilizer_kernel.cl";
  const char *kernel_function = "stabilizer_function";
  std::shared_ptr<cv::VideoCapture> getVideoCapture();
  // std::shared_ptr<ResamplerParameter> getResamplerParameterWithClockError(Eigen::VectorXd &correlation_begin, Eigen::VectorXd &correlation_end);
  std::vector<std::pair<int32_t,double>> getSyncTable(double period_in_second,int32_t width);
  std::vector<std::pair<int32_t, double>> getSyncTableOfShortVideo();
  SyncTable createSyncTable(int32_t estimate_frame, double measured_frame, double e2m_ratio);
  std::vector<std::pair<int32_t, double>> getSyncTable(double zoom, FilterPtr filter,int32_t filter_length, PointPairs &point_pairs, double duration, double length);
  double getAverageAbsoluteAngularAcceleration(const PointPairs &point_pairs, double frequency);
  PointPairs getWarpedPointPairs(double zoom, FilterPtr filter,Eigen::VectorXd &filter_strength, const PointPairs &point_pairs, int32_t start_frame, int32_t frame_length, std::vector<std::pair<int32_t,double>> &sync_table);


protected:
  std::shared_ptr<MultiThreadVideoWriter> writer_;
  std::shared_ptr<MultiThreadVideoReader> reader_;
  

  RotationPtr rotation;
  AngularVelocityPtr measured_angular_velocity;
  AngularVelocityPtr estimated_angular_velocity;
  RotationQuaternionPtr rotation_quaternion;
  //Synchronize
  AngularVelocityPtr resampled_synchronized_angular_velocity;
  ResamplerParameterPtr resampler_parameter_;

  VideoPtr video_param;
  FilterPtr filter_;
  double maximum_gradient_;
  size_t queue_size_;
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

  using UMatMap = std::map<int,UMatPtr>;
};

#endif // __VIRTUAL_GIMBAL_MANAGER_H__
