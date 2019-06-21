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

#ifndef __ROTAION_PARAM_H__
#define __ROTAION_PARAM_H__

#include <stdio.h>
#include <Eigen/Dense>
#include <memory>
#include <map>
#include "camera_information.h"
#include <iterator>
#include <list>
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
  Eigen::Vector3d getAngularVelocityVector(int frame);
  Eigen::Vector3d getAngularVelocityVector(double frame);
  Eigen::Quaterniond getAngularVelocity(int frame);
};

using AngularVelocityPtr = std::shared_ptr<AngularVelocity>;

class RotationQuaternion
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RotationQuaternion(AngularVelocityPtr angular_velocity, ResamplerParameter &resampler);
  Eigen::Quaterniond getRotationQuaternion(double time);
  Eigen::Quaterniond getCorrectionQuaternion(double time, const Eigen::VectorXd &filter_coeff);

private:
  AngularVelocityPtr angular_velocity_;
  ResamplerParameter resampler_;
  std::map<int, Eigen::Quaterniond> angle_;
  std::map<int, Eigen::MatrixXd> relative_angle_vectors;
  const Eigen::MatrixXd &getRelativeAngle(int frame, int length);
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
  virtual const Eigen::VectorXd &getFilterCoefficient() = 0;
  virtual ~Filter(){};
};

// class FIRFilter : public Filter
// {
// public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//   FIRFilter(){};
//   virtual ~FIRFilter(){};
//   virtual Eigen::VectorXd getFilterCoefficient() = 0;

// protected:
// };

class KaiserWindowFilter : public Filter
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  KaiserWindowFilter(uint32_t filter_length, uint32_t alpha);
  virtual ~KaiserWindowFilter(){}
  void setFilterCoefficient(int32_t alpha);
  const Eigen::VectorXd &getFilterCoefficient() override;
  KaiserWindowFilter & operator()(int alpha);
  size_t size();

protected:
  int32_t filter_length_;
  int32_t alpha_;
  std::map<int32_t, Eigen::VectorXd> filter_coefficients_;
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
