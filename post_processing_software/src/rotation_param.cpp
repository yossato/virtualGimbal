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

#include "rotation_param.h"

const double BaseParam::getFrequency()
{
    return frequency_;
}

const double BaseParam::getInterval()
{
    return 1. / frequency_;
}

Eigen::VectorXd BaseParam::operator()(int32_t index)
{
    return data.row(index).transpose();
}

Eigen::MatrixXd BaseParam::generateResampledData(double resampling_frequency, const resampler_parameter rparam)
{
    Eigen::MatrixXd resampled_data;
    if (resampling_frequency < std::numeric_limits<double>::epsilon())
    {
        throw "resampling_frequency is too small.";
    }
    if (rparam.start_time_second <= -std::numeric_limits<double>::epsilon())
    {
        throw "start time should be positive value.";
    }
    // Zero length means there is no specific value of resampled data length, so sets it maximum.
    if (rparam.length <= 0)
    {
        resampled_data = Eigen::MatrixXd::Zero(round(data.rows() * resampling_frequency / frequency_), data.cols()); // = Eigen::MatrixXd::Zero(round(data.rows() * resampling_frequency / frequency_), data.cols());
    }
    else
    {
        // Check length
        if (round((rparam.length / resampling_frequency + rparam.start_time_second) * frequency_) >= data.rows())
        {
            throw "Length is too large.";
        }
        resampled_data = Eigen::MatrixXd::Zero(rparam.length, data.cols());
    }
    for (int32_t frame_resampled = 0,e=resampled_data.rows(); frame_resampled < e; ++frame_resampled)
    {
        double frame_original = (frame_resampled / resampling_frequency + rparam.start_time_second) * frequency_; //ここ
        int integer_part_frame = floor(frame_original);
        double ratio = frame_original - (double)integer_part_frame;
        resampled_data.row(frame_resampled) = data.row(integer_part_frame) * (1.0 - ratio) + data.row(integer_part_frame + 1) * ratio;
    }

    return resampled_data;
}

/**
 * @brief Convert a sampling rate and generate a resampled data.
 * @param index
 * @param resampling_frequency
 * @return
 */
// Eigen::VectorXd BaseParam::operator()(int32_t index, double resampling_frequency)
// {
// if (0 == resampled_data.count(resampling_frequency))
// {
// generateResampledData(resampling_frequency);
// }
// return generateResampledData(resampling_frequency, resampler_parameter()).row(index).transpose();
// }

// Eigen::MatrixXd BaseParam::getResampledData(double resampling_frequency)
// {
//     // if (0 == resampled_data.count(resampling_frequency))
//     // {
//     //     generateResampledData(resampling_frequency);
//     // }
//     return generateResampledData(resampling_frequency, resampler_parameter());
// }

Eigen::MatrixXd BaseParam::getResampledData(double resampling_frequency, const resampler_parameter param)
{
    // if (0 == resampled_data.count(resampling_frequency))
    // {
    // generateResampledData(resampling_frequency);
    // }
    return generateResampledData(resampling_frequency, param);
}

Video::Video(double frequency)
{
    frequency_ = frequency;
}

AngularVelocity::AngularVelocity(double frequency)
{
    frequency_ = frequency;
}

Rotation::~Rotation()
{
}

Eigen::Quaterniond Rotation::getDiffQuaternion(double index)
{
    return Eigen::Quaterniond();
}
