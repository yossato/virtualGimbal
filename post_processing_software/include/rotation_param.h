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

#ifndef __ROTAION_PARAM_H__
#define __ROTAION_PARAM_H__

#include <stdio.h>
#include <Eigen/Dense>
#include <memory>
#include <map>
#include "camera_information.h"
#include <iterator>
#include <list>
#include <vector>
using QuaternionData = std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>;
using QuaternionDataPtr = std::shared_ptr<std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>>;

struct ResamplerParameter
{
  // ResamplerParameter(double frequency) : frequency(frequency), start(0.0), length(0) {}
  ResamplerParameter(double frequency, double start_time_second, double length) : frequency(frequency), start(start_time_second), length(length) {}
  double frequency;
  double start;  // Syncronized position in second.
  double length; // Length in second
};

using ResamplerParameterPtr = std::shared_ptr<ResamplerParameter>;

class BaseParam
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //    double getFrequency();
  const double getFrequency();
  const double getInterval();
  // Eigen::VectorXd operator()(int32_t index, double resampling_frequency); //クォータニオンと時はどうする？？？テンプレートクラスにする？
  Eigen::VectorXd operator()(int32_t index);
  Eigen::MatrixXd data;
  // Eigen::MatrixXd getResampledData(double resampling_frequency);
  Eigen::MatrixXd getResampledData(const ResamplerParameterPtr param);

protected:
  double frequency_;
  // std::map<double, Eigen::MatrixXd> resampled_data;
  virtual Eigen::MatrixXd generateResampledData(const ResamplerParameterPtr resample_param); // TODO: In quaternion, please implement spherical linear interpolation.
};

class Video : public BaseParam
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Video(double frequency);
  int32_t video_frames;        //! Number of frames in a video
  double rolling_shutter_time; //! Time to read all rows of CMOS sensor
  std::string video_file_name;
  CameraInformationPtr camera_info;
};

class AngularVelocity : public BaseParam
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AngularVelocity(double frequency);
  Eigen::VectorXd confidence;
  Eigen::Vector3d getAngularVelocityVector(size_t frame);
  Eigen::Vector3d getAngularVelocityVector(double frame);
  Eigen::Quaterniond getAngularVelocity(size_t frame);
  Eigen::Quaterniond getCorrectionQuaternion(double time, const Eigen::VectorXd &filter_coeff);
  double convertEstimatedToMeasuredAngularVelocityFrame(double estimate_angular_velocity_frame, std::vector<std::pair<int32_t,double>> &sync_table);
  Eigen::Quaterniond getCorrectionQuaternionFromFrame(double estimated_angular_velocity_frame, const Eigen::VectorXd &filter_coeff, std::vector<std::pair<int32_t,double>> &sync_table);
  void calculateAngleQuaternion();
  Eigen::Quaterniond quaternion(double frame);
  void getCorrectionAndRelativeQuaternion(double estimated_angular_velocity_frame, const Eigen::VectorXd &filter_coeff, std::vector<std::pair<int32_t,double>> &sync_table,  Eigen::Quaterniond& correction_quaternion, std::vector<Eigen::Quaterniond>& measured_angle_quaternions);
  double getLengthInSecond();
  int32_t getFrames();
private:
  // ResamplerParameter resampler_;
  std::map<int, Eigen::MatrixXd> relative_angle_vectors;
  std::vector<Eigen::Quaterniond> quaternion_;
  const Eigen::MatrixXd &getRelativeAngle(size_t frame, int length);
};

using AngularVelocityPtr = std::shared_ptr<AngularVelocity>;

class RotationQuaternion
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RotationQuaternion(AngularVelocityPtr angular_velocity, ResamplerParameter &resampler);
  Eigen::Quaterniond getRotationQuaternion(double time);


private:
  AngularVelocityPtr angular_velocity_;
  ResamplerParameter resampler_;
  std::map<int, Eigen::Quaterniond> angle_;

};

using RotationQuaternionPtr = std::shared_ptr<RotationQuaternion>;

/**
 * @brief The RotationData class
 * @detains クォータニオンデータを保持。必要に応じて差分のクォータニオンを差し出す。
 * これはGyro rateの角速度を格納しておく。継承クラスのVideoRateRotationにて、サンプリング周波数を変更
 */
class Rotation //なんか変だぞ？
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual Eigen::Quaterniond getDiffQuaternion(double index);
  QuaternionData quaternion;
  QuaternionData filtered_quaternion;
  virtual ~Rotation();

private:
  template <typename T_>
  void filter(QuaternionData &raw, QuaternionData &filtered, T_ &filter_coeff);

  // Diffはここで出せるようにする
};

class Filter
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Filter(){};
  // virtual const Eigen::VectorXd &getFilterCoefficient() = 0;
  virtual const Eigen::VectorXd &getFilterCoefficient(int32_t alpha) = 0;
  virtual ~Filter(){};
  virtual Filter &operator()(int filter_coefficient) = 0;
protected:
  virtual void setFilterCoefficient(int32_t alpha) = 0;

};

class NormalDistributionFilter : public Filter
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  NormalDistributionFilter();
  virtual ~NormalDistributionFilter(){};
  // const Eigen::VectorXd &getFilterCoefficient() override;
  const Eigen::VectorXd &getFilterCoefficient(int32_t half_length) override;
  NormalDistributionFilter &operator()(int32_t half_length) override;
protected:
  std::map<int32_t, Eigen::VectorXd> filter_coefficients_;
  int32_t half_length_;
  void setFilterCoefficient(int32_t half_length) override;

};

using VideoPtr = std::shared_ptr<Video>;

class VideoRateRotation : Rotation
{
  virtual Eigen::Quaterniond getDiffQuaternion(double index);

private:
  double offset;
  VideoPtr video_param;
};

using BaseParamPtr = std::shared_ptr<BaseParam>;

using RotationPtr = std::shared_ptr<Rotation>;

using VideoRateRotationPtr = std::shared_ptr<VideoRateRotation>;
using FilterPtr = std::shared_ptr<Filter>;

#endif
