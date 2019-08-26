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
        resampled_data = Eigen::MatrixXd::Zero(round((data.rows() * getInterval() - resample_param->start) * resample_param->frequency), data.cols()); // = Eigen::MatrixXd::Zero(round(data.rows() * resampling_frequency / frequency_), data.cols());
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
        assert(frame_resampled + 1 < data.rows());
        // std::cout << "frame_original:" << frame_original << " start:" << resample_param->start << " integer_part_frame" << integer_part_frame << std::endl;
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

Eigen::Vector3d AngularVelocity::getAngularVelocityVector(size_t frame)
{
    if (frame >= (size_t)data.rows())
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

Eigen::Quaterniond AngularVelocity::getAngularVelocity(size_t frame)
{
    return Vector2Quaternion<double>(getAngularVelocityVector(frame));
}

// void AngularVelocity::setResampler(ResamplerParameter &resampler){
//     resampler_ = resampler;
// }

double AngularVelocity::getLengthInSecond(){
    return data.rows()*getInterval();
}

int32_t AngularVelocity::getFrames(){
    return data.rows();
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

double AngularVelocity::convertEstimatedToMeasuredAngularVelocityFrame(double estimated_angular_velocity_frame, std::vector<std::pair<int32_t,double>> &sync_table){
 //テーブルから所望のaとbの値の計算
    auto result = std::find_if_not(sync_table.begin(),sync_table.end(),[estimated_angular_velocity_frame](std::pair<int32_t,double> x){return x.first > (int32_t) estimated_angular_velocity_frame;});
    
    int32_t x;
    int32_t x1;
    double y;
    double y1;

    // Previous section of sync_table
    if(sync_table.end() == result)
    {
        x = sync_table[0].first;
        x1 = sync_table[1].first;
        y = sync_table[0].second;
        y1 = sync_table[1].second;
    }else{
        x = result->first;
        x1 = (result+1)->first;
        y = result->second;
        y1 = (result+1)->second;
    }


    double a = (y1-y)/(x1-x);
    double b = (y*x1-x*y1)/(x1-x);

    // printf("table position:%ld a:%f b:%f\r\n",(std::distance(sync_table.begin(),result) == (int)sync_table.size() ? -1 : std::distance(sync_table.begin(),result)),a,b);




    //aとbからmeasured_angular_velocityのframeの計算
    return a * estimated_angular_velocity_frame + b;

}

Eigen::Quaterniond AngularVelocity::getCorrectionQuaternionFromFrame(   double estimated_angular_velocity_frame, 
                                                                        const Eigen::VectorXd &filter_coeff,
                                                                        std::vector<std::pair<int32_t,double>> &sync_table){
    
    double frame = convertEstimatedToMeasuredAngularVelocityFrame(estimated_angular_velocity_frame, sync_table);

    assert((frame + 1.) < (double)std::numeric_limits<size_t>::max());
    assert(frame > 0.);
    size_t integer_frame = floor(frame);

    if ((frame < 0) || (frame >= data.rows()))
    {
        std::cerr << "Waring: Frame range is out of range." << std::endl;
        return Eigen::Quaterniond(1., 0., 0., 0.);
    }
    else
    {
        // Convert time to measured anguler velocity frame position
        double ratio = frame - floor(frame);
        // std::cout << "ratio:" << ratio << std::endl;
        // std::cout << "frame:" << frame << std::endl;
        Eigen::MatrixXd first  = getRelativeAngle(integer_frame,filter_coeff.rows());
        Eigen::MatrixXd second = getRelativeAngle(integer_frame+1,filter_coeff.rows());
        // std::cout << "first:\r\n" << first << std::endl;
        // std::cout << "second:\r\n" << second << std::endl;
        // std::cout << "filter_coeff\r\n" << filter_coeff << std::endl;
        // std::cout << "first:\r\n" << first.transpose() * filter_coeff << std::endl;
        // std::cout << "second:\r\n" << second.transpose() * filter_coeff << std::endl;
        

        return Vector2Quaternion<double>(first.transpose() *  filter_coeff * (1.0 - ratio) + second.transpose() * filter_coeff * ratio).conjugate();
    }
}

Eigen::Quaterniond AngularVelocity::getCorrectionQuaternion(double time, const Eigen::VectorXd &filter_coeff)
{
    // Convert time to measured anguler velocity frame position
    const double frame = time * getFrequency();
    assert((frame + 1.) < (double)std::numeric_limits<size_t>::max());
    assert(frame > 0.);
    size_t integer_frame = floor(frame);

    if ((frame < 0) || (frame >= data.rows()))
    {
        std::cerr << "Waring: Frame range is out of range." << std::endl;
        return Eigen::Quaterniond(1., 0., 0., 0.);
    }
    else
    {
        // Convert time to measured anguler velocity frame position
        double ratio = frame - floor(frame);
        // std::cout << "ratio:" << ratio << std::endl;
        // std::cout << "frame:" << frame << std::endl;
        Eigen::MatrixXd first  = getRelativeAngle(integer_frame,filter_coeff.rows());
        Eigen::MatrixXd second = getRelativeAngle(integer_frame+1,filter_coeff.rows());
        // std::cout << "first:\r\n" << first << std::endl;
        // std::cout << "second:\r\n" << second << std::endl;
        // std::cout << "filter_coeff\r\n" << filter_coeff << std::endl;
        // std::cout << "first:\r\n" << first.transpose() * filter_coeff << std::endl;
        // std::cout << "second:\r\n" << second.transpose() * filter_coeff << std::endl;
        

        return Vector2Quaternion<double>(first.transpose() *  filter_coeff * (1.0 - ratio) + second.transpose() * filter_coeff * ratio).conjugate();
    }

}

const Eigen::MatrixXd &AngularVelocity::getRelativeAngle(size_t frame, int length)
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
    size_t center = length / 2;
    size_t r = center + 1;
    for (size_t frame_position = frame + 1; length + frame - center > frame_position; ++r, ++frame_position)
    {
        diff_rotation = (diff_rotation * Vector2Quaternion<double>(getAngularVelocityVector(frame_position))).normalized();
        rotation_vector.row(frame_position - frame + center) = Quaternion2Vector(diff_rotation,rotation_vector.row(frame_position - frame + center -1));
    }

    // r = center - 1;
    diff_rotation = Eigen::Quaterniond(1., 0., 0., 0.);
    for (size_t frame_position = frame - 1; frame - center <= frame_position; --frame_position)
    {
        diff_rotation = (diff_rotation * Vector2Quaternion<double>(getAngularVelocityVector(frame_position)).conjugate()).normalized();
        rotation_vector.row(frame_position - frame + center) = Quaternion2Vector(diff_rotation,rotation_vector.row(frame_position - frame + center + 1));
    }
    // std::cout << "rotation_vector:\r\n" << rotation_vector << std::endl;
    relative_angle_vectors[frame] = rotation_vector;
    return relative_angle_vectors[frame];
}





NormalDistributionFilter::NormalDistributionFilter(){
    // Do nothing.
}

void NormalDistributionFilter::setFilterCoefficient(int32_t half_length){
    if(half_length < 0){
        std::cerr << "half length should be larger than zero." << std::endl << std::flush;
        throw;
    }

    half_length_ = half_length;
    if(filter_coefficients_.count(half_length)){
        return;
    }else{
        filter_coefficients_[half_length] = Eigen::VectorXd::Zero(half_length*2+1);
    }

    if(0 == half_length){
        filter_coefficients_[half_length][0] = 1.0;
        return;
    }

    for(int32_t n=-half_length;n<=half_length;++n){
        filter_coefficients_[half_length][n+half_length]
        = exp(-9.0*pow((double)n/(double)half_length,2.0));
    }
    filter_coefficients_[half_length].array() /= filter_coefficients_[half_length].sum();
}

// const Eigen::VectorXd &NormalDistributionFilter::getFilterCoefficient(){
//     return filter_coefficients_[half_length_];
// }

const Eigen::VectorXd &NormalDistributionFilter::getFilterCoefficient(int32_t half_length){
    setFilterCoefficient(half_length);
    return filter_coefficients_[half_length];
}

NormalDistributionFilter &NormalDistributionFilter::operator()(int32_t half_length){
    setFilterCoefficient(half_length);
    return *this;
}