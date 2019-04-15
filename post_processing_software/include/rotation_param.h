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

using QuaternionData = std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>;
using QuaternionDataPtr = std::shared_ptr<std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>>;

struct resampler_parameter{
  resampler_parameter() : start_time_second(0.0), length(0) {}
    double start_time_second;
    int32_t length;
};

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
    Eigen::MatrixXd getResampledData(double resampling_frequency, const resampler_parameter param=resampler_parameter());

  protected:
    double frequency_;
    // std::map<double, Eigen::MatrixXd> resampled_data;
    virtual Eigen::MatrixXd generateResampledData(double resampling_frequency, const resampler_parameter param); // TODO: In quaternion, please implement spherical linear interpolation.
};

class Video : public BaseParam
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Video(double frequency);
    int32_t video_frames;        //! Number of frames in a video
    double rolling_shutter_time; //! Time to read all rows of CMOS sensor
    CameraInformationPtr camera_info;
};

class AngularVelocity : public BaseParam
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    AngularVelocity(double frequency);
    Eigen::VectorXd confidence;
};

/**
 * @brief The RotationData class
 * @detains クォータニオンデータを保持。必要に応じて差分のクォータニオンを差し出す。
 * これはGyro rateの角速度を格納しておく。継承クラスのVideoRateRotationにて、サンプリング周波数を変更
 */
class Rotation
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
    Filter(uint32_t filter_lenght);

  protected:
    int32_t filter_length_;
    std::map<int32_t, Eigen::VectorXd> filter_coefficients_;
};

class KaiserWindowFilter : public Filter
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KaiserWindowFilter(uint32_t filter_length, uint32_t alpha);
    void SetFilterCoefficient(int32_t alpha);
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
using AngularVelocityPtr = std::shared_ptr<AngularVelocity>;
using RotationPtr = std::shared_ptr<Rotation>;

using VideoRateRotationPtr = std::shared_ptr<VideoRateRotation>;
using FilterPtr = std::shared_ptr<Filter>;

#endif
