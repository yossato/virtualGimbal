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
#include "rotation_math.h"
#include <boost/math/special_functions/bessel.hpp>
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
        if (round((resample_param->start + resample_param->length) * frequency_) >= data.rows())
        {
            std::cout << "異常値を検出、デバッグ用に値を補正した。本番までに直すこと。" << std::endl;
            resample_param->start = data.rows() / frequency_ - resample_param->length;
        }
        resampled_data = Eigen::MatrixXd::Zero(round(resample_param->length * resample_param->frequency), data.cols());
    }
    for (int32_t frame_resampled = 0, e = resampled_data.rows(); frame_resampled < e; ++frame_resampled)
    {
        double frame_original = (resample_param->start + (double)frame_resampled / resample_param->frequency) * frequency_; //ここ
        int integer_part_frame = (int)frame_original;
        double ratio = frame_original - (double)integer_part_frame;
        resampled_data.row(frame_resampled) = data.row(integer_part_frame) * (1.0 - ratio) + data.row(integer_part_frame + 1) * ratio;
    }

    return resampled_data;
}

Eigen::MatrixXd BaseParam::getResampledData(const ResamplerParameterPtr resample_param)
{
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

Eigen::Vector3d AngularVelocity::getAngularVelocityVector(int frame)
{
    if ((frame < 0) || (frame >= data.rows()))
    {
        return Eigen::Vector3d(0., 0., 0.);
    }
    else
    {
        return (data.row(frame) * getInterval()).transpose();
    }
}

Eigen::Vector3d AngularVelocity::getAngularVelocityVector(double frame)
{
    if ((frame < 0) || (frame >= data.rows()))
    {
        return Eigen::Vector3d(0., 0., 0.);
    }
    else
    {
        // Convert time to measured anguler velocity frame position
        double ratio = frame - floor(frame);
        return ((data.row(frame) * (1.0 - ratio) + data.row(frame) * ratio) * getInterval()).transpose();
    }
}

Eigen::Quaterniond AngularVelocity::getAngularVelocity(int frame)
{
    return Vector2Quaternion<double>(getAngularVelocityVector(frame));
}

Rotation::~Rotation()
{
}

Eigen::Quaterniond Rotation::getDiffQuaternion(double index)
{
    return Eigen::Quaterniond();
}

RotationQuaternion::RotationQuaternion(AngularVelocityPtr angular_velocity, ResamplerParameter &resampler) : angular_velocity_(angular_velocity), resampler_(resampler)
{
    double frame = resampler_.start * angular_velocity_->getFrequency();
    angle_[(int)frame] = Eigen::Quaterniond(1, 0, 0, 0);
}

Eigen::Quaterniond RotationQuaternion::getRotationQuaternion(double time)
{
    // Convert time to measured anguler velocity frame position
    double frame = (resampler_.start + time) * angular_velocity_->getFrequency();
    assert(frame >= 0);
    int integer_frame = floor(frame);

    // std::cout << "angle_.rbegin()->first : " << angle_.rbegin()->first << std::endl;
    // std::cout << "angle_.begin()->first : " << angle_.begin()->first << std::endl;
    // std::cout << "*angle_.begin() : " << angle_.begin()->second.coeffs().transpose() << std::endl;
    // Check data availability.
    // If there is no available data, Generate it.
    if (angle_.rbegin()->first < (integer_frame + 1))
    {
        for (int i = angle_.rbegin()->first; i <= integer_frame; ++i)
        { //TODO: Angular Velocity と Angle の対応関係これでよい？1個ずれない？
            // Eigen::Vector3d vec = angular_velocity_->data.row(i).transpose() * angular_velocity_->getInterval();
            Eigen::Quaterniond diff = angular_velocity_->getAngularVelocity(i); // Vector2Quaternion<double>(angular_velocity_->data.row(i).transpose() * angular_velocity_->getInterval());
            angle_[i + 1] = (angle_[i] * diff).normalized();
            // std::cout << angle_[i].coeffs().transpose() << std::endl;
            // std::cout << angle_[i+1].coeffs().transpose() << std::endl << std::flush;
        }
    }
    else if (integer_frame < angle_.begin()->first)
    {
        assert(integer_frame >= 0);
        for (int i = angle_.begin()->first - 1; integer_frame <= i; --i)
        {
            Eigen::Quaterniond diff = angular_velocity_->getAngularVelocity(i); //Vector2Quaternion<double>(angular_velocity_->data.row(i).transpose() * angular_velocity_->getInterval());
            angle_[i] = (angle_[i + 1] * diff.conjugate()).normalized();
        }
    }
    // Return slerped quaternion.
    return angle_[integer_frame].slerp(frame - integer_frame, angle_[integer_frame + 1]);
}

Eigen::Quaterniond RotationQuaternion::getCorrectionQuaternion(double time, const Eigen::VectorXd &filter_coeff)
{
    // Convert time to measured anguler velocity frame position
    const double frame = (resampler_.start + time) * angular_velocity_->getFrequency();
    int integer_frame = floor(frame);

    if ((frame < 0) || (frame >= angular_velocity_->data.rows()))
    {
        std::cout << "Waring: Frame range is out of range." << std::endl;
        return Eigen::Quaterniond(1., 0., 0., 0.);
    }
    else
    {
        // Convert time to measured anguler velocity frame position
        double ratio = frame - floor(frame);
        Eigen::MatrixXd first  = getRelativeAngle(integer_frame,filter_coeff.rows());
        Eigen::MatrixXd second = getRelativeAngle(integer_frame+1,filter_coeff.rows());
        return Vector2Quaternion<double>(first.transpose() *  filter_coeff * (1.0 - ratio) + second.transpose() * filter_coeff * ratio).conjugate();
    }

}

const Eigen::MatrixXd &RotationQuaternion::getRelativeAngle(int frame, int length)
{
    // Read the angle from a buffer if available.
    if (relative_angle_vectors.count(frame))
    {
        if (relative_angle_vectors[frame].rows() != length)
        {
            relative_angle_vectors.erase(frame);
        }
        else
        {
            return relative_angle_vectors[frame];
        }
    }

    // It is not available, create it.
    Eigen::Quaterniond diff_rotation(1., 0., 0., 0.);
    Eigen::MatrixXd rotation_vector = Eigen::MatrixXd::Zero(length, 3);
    int center = length / 2;
    int r = center + 1;
    for (int frame_position = frame + 1; length + frame - center > frame_position; ++r, ++frame_position)
    {
        diff_rotation = diff_rotation * Vector2Quaternion<double>(angular_velocity_->getAngularVelocityVector(frame_position));
        rotation_vector.row(frame_position - frame + center) = Quaternion2Vector(diff_rotation);
    }

    // r = center - 1;
    diff_rotation = Eigen::Quaterniond(1., 0., 0., 0.);
    for (int frame_position = frame - 1; frame - center <= frame_position; --frame_position)
    {
        diff_rotation = diff_rotation * Vector2Quaternion<double>(angular_velocity_->getAngularVelocityVector(frame_position));
        rotation_vector.row(frame_position - frame + center) = Quaternion2Vector(diff_rotation);
    }
    relative_angle_vectors[frame] = rotation_vector;
    return relative_angle_vectors[frame];
}



KaiserWindowFilter::KaiserWindowFilter(uint32_t filter_length, uint32_t alpha) : Filter(), filter_length_(filter_length)
{
    assert(filter_length_ % 2);// Must be odd
    setFilterCoefficient(alpha);
}

const Eigen::VectorXd &KaiserWindowFilter::getFilterCoefficient(){
    return filter_coefficients_[alpha_];
}

void KaiserWindowFilter::setFilterCoefficient(int32_t alpha){
    alpha_ = alpha;
    if(filter_coefficients_.count(alpha_)){ // Filter cofficient already exists.
        return;
    }
   
    int32_t L = filter_coefficients_[alpha].rows()/2;
    for(int32_t n=-L,e=L;n<=e;++n){
        filter_coefficients_[alpha][n+L] = boost::math::cyl_bessel_i(0.0,alpha_*sqrt(1.0-pow((double)n/(double)L,2.0)))
                /boost::math::cyl_bessel_i(0.0,alpha_);
    }
}

KaiserWindowFilter  &KaiserWindowFilter::operator()(int alpha){
    setFilterCoefficient(alpha);
    return *this;
}

size_t  KaiserWindowFilter::size(){
    return filter_length_;
}