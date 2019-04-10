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

void BaseParam::generateResampledData(double resampling_frequency)
{
    if (resampling_frequency < std::numeric_limits<double>::epsilon())
    {
        throw "resampling_frequency is too small.";
    }
    resampled_data[resampling_frequency] = Eigen::MatrixXd::Zero(round(data.rows() * resampling_frequency / frequency_), data.cols());// = Eigen::MatrixXd::Zero(round(data.rows() * resampling_frequency / frequency_), data.cols());
    Eigen::MatrixXd &ref = resampled_data[resampling_frequency];
    for (int32_t frame = 0, end = ref.rows(); frame < end; ++frame)
    {
        double resampled_frame = frame * frequency_ / resampling_frequency;
        int integer_part_frame = floor(resampled_frame);
        double ratio = resampled_frame - (double)integer_part_frame;
        ref.row(frame) = data.row(integer_part_frame) * (1.0 - ratio) + data.row(integer_part_frame + 1) * ratio;
    }
    // std::cout << "end!" << std::endl;
}

/**
 * @brief Convert a sampling rate and generate a resampled data.
 * @param index
 * @param resampling_frequency
 * @return
 */
Eigen::VectorXd BaseParam::operator()(int32_t index, double resampling_frequency)
{
    if (0 == resampled_data.count(resampling_frequency))
    {
        generateResampledData(resampling_frequency);
    }
    return resampled_data[resampling_frequency].row(index).transpose();
}

Eigen::MatrixXd &BaseParam::getResampledData(double resampling_frequency)
{
    if (0 == resampled_data.count(resampling_frequency))
    {
        generateResampledData(resampling_frequency);
    }
    return resampled_data[resampling_frequency];
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
