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

Eigen::MatrixXd BaseParam::generateResampledData(const ResamplerParameterPtr resample_param)
{
    Eigen::MatrixXd resampled_data;
    assert(resample_param->frequency > std::numeric_limits<double>::epsilon());
    assert(resample_param->start > -std::numeric_limits<double>::epsilon());

    // Zero length means there is no specific value of resampled data length, so sets it maximum.
    if (resample_param->length < std::numeric_limits<double>::epsilon())
    {
        resampled_data = Eigen::MatrixXd::Zero(round(data.rows() * resample_param->frequency / frequency_), data.cols()); // = Eigen::MatrixXd::Zero(round(data.rows() * resampling_frequency / frequency_), data.cols());
        resample_param->length = resampled_data.rows() / resample_param->frequency;
    }
    else
    {
        // Check length
        // assert(round((resample_param->start + resample_param->length) * frequency_)  < data.rows());//ここでうまく行かない
        if(round((resample_param->start + resample_param->length) * frequency_)  >= data.rows()){
            std::cout << "i異常値を検出認め、デバッグ用に値を補正した。本番までに直すこと。" << std::endl;
            resample_param->start = data.rows()/frequency_-resample_param->length;
        }
        resampled_data = Eigen::MatrixXd::Zero(round(resample_param->length * resample_param->frequency), data.cols());
    }
    for (int32_t frame_resampled = 0,e=resampled_data.rows(); frame_resampled < e; ++frame_resampled)
    {
        double frame_original = (resample_param->start + (double)frame_resampled / resample_param->frequency) * frequency_; //ここ
        int integer_part_frame = (int)frame_original;
        double ratio = frame_original - (double)integer_part_frame;
        resampled_data.row(frame_resampled) = data.row(integer_part_frame) * (1.0 - ratio) + data.row(integer_part_frame + 1) * ratio;
    }

    return resampled_data;
}

/**
 * @brief Convert a sampling rate and generate a resampled data.
 * @resample_param index
 * @resample_param resampling_frequency
 * @return
 */
// Eigen::VectorXd BaseParam::operator()(int32_t index, double resampling_frequency)
// {
// if (0 == resampled_data.count(resampling_frequency))
// {
// generateResampledData(resampling_frequency);
// }
// return generateResampledData(resampling_frequency, ResamplerParameter()).row(index).transpose();
// }

// Eigen::MatrixXd BaseParam::getResampledData(double resampling_frequency)
// {
//     // if (0 == resampled_data.count(resampling_frequency))
//     // {
//     //     generateResampledData(resampling_frequency);
//     // }
//     return generateResampledData(resampling_frequency, ResamplerParameter());
// }

Eigen::MatrixXd BaseParam::getResampledData(const ResamplerParameterPtr resample_param)
{
    // if (0 == resampled_data.count(resampling_frequency))
    // {
    // generateResampledData(resampling_frequency);
    // }
    return generateResampledData(resample_param);
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
