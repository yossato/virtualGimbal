#ifndef __ROTAION_PARAM_H__
#define __ROTAION_PARAM_H__

#include <stdio.h>
#include <Eigen/Dense>

using AngleQuaternion = std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>;
using AngleQuaternionPtr = std::shared_ptr<std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>>;

class RotationParam
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double video_frame_rate;
    double gyro_sampling_rate;
};

class RotationData
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RotationData();
    AngleQuaternionPtr quaternion;
    AngleQuaternionPtr filtered_quaternion; 
};

// class RawRotationData : RotationData
// {
//   public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//     RawRotationData();
// };

// class FilteredRotationData : RotaionData
// {
//     public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//     FilteredRotationData();
// };

#endif