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
    assert(resample_param->original_data_center >= (resample_param->resampled_data_length/2));
    assert(resample_param->resampled_data_length > 0);

    // size_t resampled_data_length = (int)std::round(resample_param->length / frequency_ * resample_param->frequency);
    // if(!(resampled_data_length % 2)) resampled_data_length += 1; //resampled_data_length must be odd.
    resampled_data = Eigen::MatrixXd::Zero(resample_param->resampled_data_length, data.cols());

    int32_t half_resampled_data_rows = resampled_data.rows()/2;
    for (int32_t resampled_frame_diff = -half_resampled_data_rows, e = resampled_data.rows() - half_resampled_data_rows; resampled_frame_diff < e; ++resampled_frame_diff)
    {
        
        double frame_original = resample_param->original_data_center + (double)resampled_frame_diff / resample_param->frequency * frequency_;
        int integer_part_frame = (int)frame_original;
        double ratio = frame_original - (double)integer_part_frame;
        
        assert(integer_part_frame >= 0);
        assert(integer_part_frame < data.rows());

        size_t resampled_frame = resampled_frame_diff + half_resampled_data_rows;
        resampled_data.row(resampled_frame) = data.row(integer_part_frame) * (1.0 - ratio) + data.row(integer_part_frame + 1) * ratio;
    }

    return resampled_data;
}

Eigen::MatrixXd BaseParam::generateResampledData(const int32_t length, const double ratio, const double frame_position)
{
    Eigen::MatrixXd resampled_data = Eigen::MatrixXd::Zero(length, data.cols());

    int32_t half_length = length/2;
    for (int32_t diff = - half_length, e = half_length; diff <= e; ++diff)
    {
        
        double this_frame_position = (frame_position + diff) / ratio;
        int32_t this_frame_integer_position = (int32_t)this_frame_position;
        double s = this_frame_position - (double)this_frame_integer_position;
        
        assert(this_frame_integer_position >= 0);
        assert(this_frame_integer_position+1 < data.rows());

        size_t resampled_frame = diff + half_length;
        resampled_data.row(resampled_frame) = data.row(this_frame_integer_position) * (1.0 - s) + data.row(this_frame_integer_position + 1) * s;
    }

    return resampled_data;
}

Eigen::MatrixXd BaseParam::generateResampledData(const double ratio)
{

    assert(ratio > std::numeric_limits<double>::epsilon());

    Eigen::MatrixXd resampled_data = Eigen::MatrixXd::Zero((int)std::round(data.rows() * ratio), data.cols());

    // int32_t half_resampled_data_rows = resampled_data.rows()/2;
    for (int32_t resampled_frame = 0, e = resampled_data.rows(); resampled_frame < e; ++resampled_frame)
    {
        
        double this_frame = (double)resampled_frame / ratio;
  

        int32_t this_frame_integer_position = (int32_t)this_frame;
        double s = this_frame - (double)this_frame_integer_position;
        
        assert(this_frame_integer_position >= 0);
        assert(this_frame_integer_position + 1 < data.rows());

        resampled_data.row(resampled_frame) = data.row(this_frame_integer_position) * (1.0 - s) + data.row(this_frame_integer_position + 1) * s;
    }

    return resampled_data;
}

Eigen::MatrixXd BaseParam::getResampledData(const ResamplerParameterPtr resample_param)
{
    return generateResampledData(resample_param);
}

Eigen::MatrixXd BaseParam::getResampledData(const double ratio)
{
    return generateResampledData(ratio);
}

Eigen::MatrixXd BaseParam::getResampledData(const int32_t length, const double ratio, const double frame_position)
{
    return generateResampledData(length, ratio, frame_position);
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

// RotationQuaternion::RotationQuaternion(AngularVelocityPtr angular_velocity, ResamplerParameter &resampler) : angular_velocity_(angular_velocity), resampler_(resampler)
// {
//     double frame = resampler_.start * angular_velocity_->getFrequency();
//     angle_[(int)frame] = Eigen::Quaterniond(1, 0, 0, 0);
// }

// Eigen::Quaterniond RotationQuaternion::getRotationQuaternion(double time)
// {
//     // Convert time to measured anguler velocity frame position
//     double frame = (resampler_.start + time) * angular_velocity_->getFrequency();
//     assert(frame >= 0);
//     int integer_frame = floor(frame);

//     // std::cout << "angle_.rbegin()->first : " << angle_.rbegin()->first << std::endl;
//     // std::cout << "angle_.begin()->first : " << angle_.begin()->first << std::endl;
//     // std::cout << "*angle_.begin() : " << angle_.begin()->second.coeffs().transpose() << std::endl;
//     // Check data availability.
//     // If there is no available data, Generate it.
//     if (angle_.rbegin()->first < (integer_frame + 1))
//     {
//         for (int i = angle_.rbegin()->first; i <= integer_frame; ++i)
//         { //TODO: Angular Velocity と Angle の対応関係これでよい？1個ずれない？
//             // Eigen::Vector3d vec = angular_velocity_->data.row(i).transpose() * angular_velocity_->getInterval();
//             Eigen::Quaterniond diff = angular_velocity_->getAngularVelocity(i); // Vector2Quaternion<double>(angular_velocity_->data.row(i).transpose() * angular_velocity_->getInterval());
//             angle_[i + 1] = (angle_[i] * diff).normalized();
//             // std::cout << angle_[i].coeffs().transpose() << std::endl;
//             // std::cout << angle_[i+1].coeffs().transpose() << std::endl << std::flush;
//         }
//     }
//     else if (integer_frame < angle_.begin()->first)
//     {
//         assert(integer_frame >= 0);
//         for (int i = angle_.begin()->first - 1; integer_frame <= i; --i)
//         {
//             Eigen::Quaterniond diff = angular_velocity_->getAngularVelocity(i); //Vector2Quaternion<double>(angular_velocity_->data.row(i).transpose() * angular_velocity_->getInterval());
//             angle_[i] = (angle_[i + 1] * diff.conjugate()).normalized();
//         }
//     }
//     // Return slerped quaternion.
//     return angle_[integer_frame].slerp(frame - integer_frame, angle_[integer_frame + 1]);
// }

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

void AngularVelocity::calculateAngleQuaternion()
{
    quaternion_.clear();
    Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
    for(size_t frame=0;frame<(size_t)data.rows();++frame)
    {
        q = (q * Vector2Quaternion<double>(getAngularVelocityVector(frame))).normalized();
        quaternion_.push_back(q);
    }
}

Eigen::Quaterniond AngularVelocity::quaternion(double frame)
{
    size_t integer_frame = floor(frame);
    if ((frame < 0) || (frame >= (data.rows()-1)))
    {
        std::cerr << "Error: Frame range is out of range." << std::endl;
        return Eigen::Quaterniond(1.0, 0,0,0);
    }
    else
    {
        double ratio = frame - integer_frame;
        return quaternion_[integer_frame].slerp(ratio,quaternion_[integer_frame+1]);
    }
}

double AngularVelocity::getStabilizedQuaternion(double estimated_angular_velocity_frame, 
                                                        const Eigen::VectorXd &filter_coeff, 
                                                        std::vector<std::pair<int32_t,double>> &sync_table, 
                                                        Eigen::Quaterniond& stabilized_angle_quaternion)
{
    double frame = convertEstimatedToMeasuredAngularVelocityFrame(estimated_angular_velocity_frame, sync_table);
    assert((frame + 1.) < (double)std::numeric_limits<size_t>::max());
    assert(frame > 0.);
    // size_t integer_frame = floor(frame);

    if ((frame < 0) || (frame >= data.rows()))
    {
        std::cerr << "Error: Frame range is out of range." << std::endl;
        return std::numeric_limits<double>::quiet_NaN() ;
    }
    else
    {
        Eigen::MatrixXd relative_angle_vector(filter_coeff.rows(),3);
        Eigen::Quaterniond origin_quaternion = quaternion(frame);
        Eigen::Quaterniond conjugate_origin_quaternion = origin_quaternion.conjugate();


        // Convert time to measured anguler velocity frame position
        int length = filter_coeff.rows();
        // double ratio = frame - floor(frame);

        Eigen::MatrixXd rotation_vector = Eigen::MatrixXd::Zero(length, 3);
        int center = length / 2;
        for (int frame_position = 1; length - center > frame_position; ++frame_position)
        {
            rotation_vector.row(frame_position + center) = Quaternion2Vector(quaternion((double)frame_position + frame) * conjugate_origin_quaternion, rotation_vector.row(frame_position + center -1));
        }

        for (int frame_position = - 1; - center <= frame_position; --frame_position)
        {
            rotation_vector.row(frame_position + center) = Quaternion2Vector(quaternion((double)frame_position + frame) * conjugate_origin_quaternion , rotation_vector.row(frame_position + center +1));
        }

        // std::cout << "rotation_vector" << std::endl << rotation_vector << std::endl;

        // std::cout << "original_quaternion:" << origin_quaternion.coeffs().transpose() << std::endl; 
        // std::cout << "part:" << Vector2Quaternion<double>(rotation_vector.transpose() *  filter_coeff).conjugate().normalized().coeffs().transpose() << std::endl; 
        // std::cout << "stabilized_angle_quaternion:" << (Vector2Quaternion<double>(rotation_vector.transpose() *  filter_coeff).conjugate().normalized() * origin_quaternion).coeffs().transpose() << std::endl; 

        Eigen::Quaterniond diff_angle_quaternion = Vector2Quaternion<double>(rotation_vector.transpose() *  filter_coeff).normalized();
        stabilized_angle_quaternion = diff_angle_quaternion * origin_quaternion;
        double diff_angle =  acos(diff_angle_quaternion.w()) * 2.0;
        return diff_angle;
    }
}

void AngularVelocity::getCorrectionMatrices(  const Eigen::Quaterniond& stabilized_angle_quaternion,
                                            int frame,
                                            int height,
                                            double line_delay_in_video_frame,
                                            std::vector<std::pair<int32_t,double>> &sync_table, 
                                            std::vector<float> &stabilized_angle_matrices)
{
    double begin_line_in_gyro_frame = convertEstimatedToMeasuredAngularVelocityFrame(frame - line_delay_in_video_frame*height, sync_table);
    double end_line_in_gyro_frame = convertEstimatedToMeasuredAngularVelocityFrame(frame + line_delay_in_video_frame*height, sync_table);

    if ((begin_line_in_gyro_frame < 0) || (begin_line_in_gyro_frame >= data.rows()) || (end_line_in_gyro_frame < 0) || (end_line_in_gyro_frame >= data.rows()))
    {
        std::cerr << "Error: Frame range is out of range." << std::endl;
        return;
    }
    else
    {
        stabilized_angle_matrices.resize(height * 3 * 3); // height * (3x3 matirx)
        for(int row = 0; row<height; ++ row)
        {
            double line_position_in_video_frame = frame + line_delay_in_video_frame*(row - height * 0.5);
            double line_position_in_gyro_frame = convertEstimatedToMeasuredAngularVelocityFrame(line_position_in_video_frame, sync_table);
            
            Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(&(stabilized_angle_matrices[row * 3 * 3]),3,3) 
            // = (quaternion(line_position_in_gyro_frame) * stabilized_angle_quaternion.conjugate()).matrix().cast<float>();
            // = (stabilized_angle_quaternion * quaternion(line_position_in_gyro_frame).conjugate()).matrix().cast<float>();
            // = (quaternion(line_position_in_gyro_frame).conjugate() * stabilized_angle_quaternion).matrix().cast<float>();
            = (stabilized_angle_quaternion.conjugate() * quaternion(line_position_in_gyro_frame)).matrix().cast<float>();
        }
    }
}

/*Eigen::Quaterniond AngularVelocity::getCorrectionQuaternion(double time, const Eigen::VectorXd &filter_coeff)
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

}*/

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

    // TODO: Memory limitation, limit relative_angle_vectors's size

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