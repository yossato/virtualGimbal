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
#include "virtual_gimbal_manager.h"

using namespace cv;
using namespace std;

VirtualGimbalManager::VirtualGimbalManager()
{
}

VirtualGimbalManager::VirtualGimbalManager(size_t queue_size) : queue_size_(queue_size)
{
}

std::string VirtualGimbalManager::getVideoSize(const char *videoName)
{
    std::shared_ptr<cv::VideoCapture> Capture = std::make_shared<cv::VideoCapture>(videoName); //動画をオープン
    assert(Capture->isOpened());
    std::string videoSize = std::to_string((int)Capture->get(cv::CAP_PROP_FRAME_WIDTH)) + std::string("x") + std::to_string((int)Capture->get(cv::CAP_PROP_FRAME_HEIGHT));
    return videoSize;
}

void VirtualGimbalManager::setVideoParam(const char *file_name, CameraInformationPtr info)
{
    std::shared_ptr<cv::VideoCapture> capture = std::make_shared<cv::VideoCapture>(file_name); //動画をオープン
    if (!capture->isOpened())
    {
        throw "Video not found.";
    }
    video_param.reset(new Video(capture->get(cv::CAP_PROP_FPS)));
    video_param->video_frames = capture->get(cv::CAP_PROP_FRAME_COUNT);
    video_param->rolling_shutter_time = 0.0;
    video_param->camera_info = info;
    video_param->video_file_name = file_name;
}

void VirtualGimbalManager::setMeasuredAngularVelocity(const char *file_name, CameraInformationPtr info)
{
    try
    {
        measured_angular_velocity.reset(new AngularVelocity(readSamplingRateFromJson(file_name)));
        measured_angular_velocity->data = readAngularVelocityFromJson(file_name);
    }
    catch (std::string e)
    {
        std::cerr << "Error: " << e << std::endl;
        std::exit(EXIT_FAILURE);
    }
    if (info)
    {
        rotateAngularVelocity(measured_angular_velocity->data, info->sd_card_rotation_);
    }
}

/**
 * @brief For angular velocity from chess board 
 **/
void VirtualGimbalManager::setEstimatedAngularVelocity(Eigen::MatrixXd &angular_velocity, Eigen::VectorXd confidence, double frequency)
{
    if (0.0 == frequency)
    {
        estimated_angular_velocity = std::make_shared<AngularVelocity>(video_param->getFrequency());
    }
    else
    {
        estimated_angular_velocity = std::make_shared<AngularVelocity>(frequency);
    }
    estimated_angular_velocity->confidence = confidence;
    estimated_angular_velocity->data = angular_velocity;
}

void VirtualGimbalManager::setRotation(const char *file_name, CameraInformation &cameraInfo)
{
    rotation.reset(new Rotation());
}

// double VirtualGimbalManager::getMeasuredFramePositionFrom(int32_t estimated_frame_position, )

double VirtualGimbalManager::getMeasuredFramePositionFrom(int32_t estimated_frame_position, int32_t length)
{


    double e2m = measured_angular_velocity->getFrequency()/estimated_angular_velocity->getFrequency();
    double m2e = 1./e2m;
    Eigen::VectorXd correlation_coefficients;

    assert(length % 2); // Odd

    int32_t half_length = length / 2;

    int32_t begin = estimated_frame_position - half_length;
    assert(begin >= 0);
    assert(begin+length<=estimated_angular_velocity->data.rows());
    Eigen::MatrixXd particial_estimated_angular_velocity = estimated_angular_velocity->data.block(begin, 0, length, estimated_angular_velocity->data.cols());
    Eigen::VectorXd particial_confidence = estimated_angular_velocity->confidence.block(begin, 0, length, estimated_angular_velocity->confidence.cols());

    {
        Eigen::MatrixXd measured_angular_velocity_resampled = measured_angular_velocity->getResampledData(m2e);
        //   std::cout << "measured_angular_velocity_resampled.row(8000):" << measured_angular_velocity_resampled.row(8000) << std::endl; 

        int32_t diff = measured_angular_velocity_resampled.rows() - particial_estimated_angular_velocity.rows();

        {
            LoggingDouble d;
            for(int r=0;r<particial_estimated_angular_velocity.rows();++r)
            {
                d["Frame"].push_back((double)r);
                d["x"].push_back(particial_estimated_angular_velocity(r,0));
                d["y"].push_back(particial_estimated_angular_velocity(r,1));
                d["z"].push_back(particial_estimated_angular_velocity(r,2));

            }
            std::string time_stamp = DataCollection::getSystemTimeStamp();
            DataCollection collection("latest_particial_estimated_angular_velocity.csv");
            collection.set(d);
        }
        {
            LoggingDouble d;
            for(int r=0;r<measured_angular_velocity_resampled.rows();++r)
            {
                d["Frame"].push_back((double)r);
                // std::cout << measured_angular_velocity_resampled.row(r) << std::endl;
                d["x"].push_back(measured_angular_velocity_resampled(r,0));
                d["y"].push_back(measured_angular_velocity_resampled(r,1));
                d["z"].push_back(measured_angular_velocity_resampled(r,2));

            }
            std::string time_stamp = DataCollection::getSystemTimeStamp();
            DataCollection collection("latest_measured_angular_velocity_resampled.csv");
            collection.set(d);
        }


        assert(diff > 0); // Measured angular velocity must be longer than estimated one.
    
        int32_t number_of_data = particial_confidence.cast<int>().array().sum();
        if (0 == number_of_data)
        {
            std::cerr << "There is no valid data in estimated angular velocity from video." << std::endl;
            return std::numeric_limits<double>::quiet_NaN();
        }

        correlation_coefficients = Eigen::VectorXd (diff + 1);
        for (int32_t frame = 0, end = correlation_coefficients.rows(); frame < end; ++frame)
        {
            correlation_coefficients[frame] = ((measured_angular_velocity_resampled.block(frame, 0, particial_estimated_angular_velocity.rows(), particial_estimated_angular_velocity.cols()) 
            - particial_estimated_angular_velocity).array().colwise() * particial_confidence.array()).abs().sum() / (double)number_of_data;

            // if (frame % 100 == 0)
            // {
            //     printf("\r%d / %d", frame, diff);
            //     std::cout << std::flush;
            // }
        }
    }

    //最小値サブピクセル推定
    Eigen::VectorXd::Index minimum_correlation_frame_in_estimated_frame;
    
    correlation_coefficients.minCoeff(&minimum_correlation_frame_in_estimated_frame);
    int32_t synchronized_frame_in_estimated_frame = minimum_correlation_frame_in_estimated_frame + half_length;
    {
        LoggingDouble d;
        for(int r=0;r<correlation_coefficients.rows();++r)
        {
            d["Frame"].push_back((double)r);
            d["cc"].push_back(correlation_coefficients(r));

        }
        std::string time_stamp = DataCollection::getSystemTimeStamp();
        DataCollection collection("latest_correlation_coefficients.csv");
        collection.set(d);
    }
        
    // return synchronized_frame_in_estimated_frame  * e2m;
    if ((minimum_correlation_frame_in_estimated_frame == 0) || (minimum_correlation_frame_in_estimated_frame == (correlation_coefficients.rows() - 1)))
    { //位置が最初のフレームで一致している場合
        return synchronized_frame_in_estimated_frame  * e2m;
    }    
    double minimum_correlation_subframe_in_estimated_frame = 0.0;
    std::cout << "synchronized_frame_in_estimated_frame:" << synchronized_frame_in_estimated_frame << std::endl;
    double min_value = std::numeric_limits<double>::max();
    int32_t number_of_data = particial_confidence.cast<int>().array().sum();
    for (double sub_frame = -100.0; sub_frame <= 100.0; sub_frame += 0.01)
    {
        double frame_position = synchronized_frame_in_estimated_frame + sub_frame;
        Eigen::MatrixXd measured_angular_velocity_resampled = measured_angular_velocity->getResampledData(length,m2e,frame_position);

        assert(measured_angular_velocity_resampled.rows() == particial_estimated_angular_velocity.rows());
        
        double value = ((measured_angular_velocity_resampled - particial_estimated_angular_velocity).array().colwise() * particial_confidence.array()).abs().sum() / (double)number_of_data;
        if (min_value > value)
        {
            min_value = value;
            minimum_correlation_subframe_in_estimated_frame = frame_position;
        }
    }
    std::cout << "min_value:" << min_value << std::endl;
    std::cout << "minimum_correlation_subframe:" << minimum_correlation_subframe_in_estimated_frame << std::endl;

    return minimum_correlation_subframe_in_estimated_frame * e2m;
    
}

Eigen::VectorXd VirtualGimbalManager::getCorrelationCoefficient(int32_t estimated_center_frame, int32_t length, double frequency)
{
    if (frequency <= std::numeric_limits<double>::epsilon())
    {
        frequency = video_param->getFrequency();
    }

    assert(length % 2); // Odd

    
    int32_t half_length = length / 2;
    int32_t begin = estimated_center_frame - half_length;
    assert(begin >= 0);
    Eigen::MatrixXd particial_estimated_angular_velocity = estimated_angular_velocity->data.block(begin, 0, length, estimated_angular_velocity->data.cols());
    Eigen::VectorXd particial_confidence = estimated_angular_velocity->confidence.block(begin, 0, length, estimated_angular_velocity->confidence.cols());

    
    Eigen::MatrixXd measured_angular_velocity_resampled = measured_angular_velocity->getResampledData(frequency);
    int32_t diff = measured_angular_velocity_resampled.rows() - particial_estimated_angular_velocity.rows();

    assert(diff > 0); // Measured angular velocity must be longer than estimated one.

    


    int32_t number_of_data = particial_confidence.cast<int>().array().sum();
    if (0 == number_of_data)
    {
        return Eigen::VectorXd::Constant(diff + 1, std::numeric_limits<double>::max());
    }


    Eigen::VectorXd correlation_coefficients(diff + 1);
    for (int32_t frame = 0, end = correlation_coefficients.rows(); frame < end; ++frame)
    {
        correlation_coefficients[frame] = ((measured_angular_velocity_resampled.block(frame, 0, particial_estimated_angular_velocity.rows(), particial_estimated_angular_velocity.cols()) 
        - particial_estimated_angular_velocity).array().colwise() * particial_confidence.array()).abs().sum() / (double)number_of_data;

        if (frame % 100 == 0)
        {
            printf("\r%d / %d", frame, diff);
            std::cout << std::flush;
        }
    }
    return correlation_coefficients;



    
}

double VirtualGimbalManager::getSubframeOffsetInSecond(Eigen::VectorXd &correlation_coefficients, int32_t begin, int32_t length, double frequency)
{
    // Set default value if these are not set.
    if (0 == length)
    {
        length = estimated_angular_velocity->data.rows();
    }
    if (frequency < std::numeric_limits<double>::epsilon())
    {
        frequency = video_param->getFrequency();
    }

    // Error check
    assert(length <= estimated_angular_velocity->data.rows());

    std::vector<double> vec_correlation_cofficients(correlation_coefficients.rows());
    Eigen::Map<Eigen::VectorXd>(vec_correlation_cofficients.data(), correlation_coefficients.rows(), 1) = correlation_coefficients;
    int32_t minimum_correlation_frame = std::distance(vec_correlation_cofficients.begin(), min_element(vec_correlation_cofficients.begin(), vec_correlation_cofficients.end()));

    Eigen::MatrixXd particial_estimated_angular_velocity = estimated_angular_velocity->data.block(begin, 0, length, estimated_angular_velocity->data.cols());
    Eigen::VectorXd particial_confidence = estimated_angular_velocity->confidence.block(begin, 0, length, estimated_angular_velocity->confidence.cols());

    //最小値サブピクセル推定
    double minimum_correlation_subframe = 0.0;
    if (minimum_correlation_frame == 0)
    { //位置が最初のフレームで一致している場合
        minimum_correlation_subframe = 0.0;
    }
    else if (minimum_correlation_frame == (correlation_coefficients.rows() - 2)) //なんで2?
    {                                                                            //末尾
        minimum_correlation_subframe = (double)(correlation_coefficients.rows() - 2);
    }
    else
    {
        std::cout << "minimum_correlation_frame" << minimum_correlation_frame << std::endl;
        double min_value = std::numeric_limits<double>::max();
        int32_t number_of_data = particial_confidence.cast<int>().array().sum();
        for (double sub_frame = -2.0; sub_frame <= 2.0; sub_frame += 0.0001)
        {
            Eigen::MatrixXd measured_angular_velocity_resampled = measured_angular_velocity->getResampledData(std::make_shared<ResamplerParameter>(frequency, (sub_frame + minimum_correlation_frame) / frequency, estimated_angular_velocity->getInterval() * particial_estimated_angular_velocity.rows()));
            assert(measured_angular_velocity_resampled.rows() == particial_estimated_angular_velocity.rows());
            double value = ((measured_angular_velocity_resampled - particial_estimated_angular_velocity).array().colwise() * particial_confidence.array()).abs().sum() / (double)number_of_data;
            if (min_value > value)
            {
                min_value = value;
                minimum_correlation_subframe = sub_frame;
            }
        }
        std::cout << "min_value:" << min_value << std::endl;
        std::cout << "minimum_correlation_subframe:" << minimum_correlation_subframe;
    }
    minimum_correlation_subframe += (double)minimum_correlation_frame;
    std::cout << std::endl
              << minimum_correlation_subframe << std::endl;

    return minimum_correlation_subframe / frequency - (estimated_angular_velocity->getInterval() - measured_angular_velocity->getInterval()) * 0.5;
}

double VirtualGimbalManager::getSubframeOffset(Eigen::VectorXd &correlation_coefficients, int32_t center, int32_t length, double frequency)
{
    // Set default value if these are not set.
    if (0 == length)
    {
        length = estimated_angular_velocity->data.rows();
    }
    if (frequency < std::numeric_limits<double>::epsilon())
    {
        frequency = video_param->getFrequency();
    }

    assert(length%2);

    int32_t begin = center - length/2;

    // Error check
    assert(length <= estimated_angular_velocity->data.rows());

    std::vector<double> vec_correlation_cofficients(correlation_coefficients.rows());
    Eigen::Map<Eigen::VectorXd>(vec_correlation_cofficients.data(), correlation_coefficients.rows(), 1) = correlation_coefficients;
    int32_t minimum_correlation_frame = std::distance(vec_correlation_cofficients.begin(), min_element(vec_correlation_cofficients.begin(), vec_correlation_cofficients.end()));

    Eigen::MatrixXd particial_estimated_angular_velocity = estimated_angular_velocity->data.block(begin, 0, length, estimated_angular_velocity->data.cols());
    Eigen::VectorXd particial_confidence = estimated_angular_velocity->confidence.block(begin, 0, length, estimated_angular_velocity->confidence.cols());

    //最小値サブピクセル推定
    double minimum_correlation_subframe = 0.0;
    if (minimum_correlation_frame == 0)
    { //位置が最初のフレームで一致している場合
        minimum_correlation_subframe = 0.0;
    }
    else if (minimum_correlation_frame == (correlation_coefficients.rows() - 2)) //なんで2?
    {                                                                            //末尾
        minimum_correlation_subframe = (double)(correlation_coefficients.rows() - 2);
    }
    else
    {
        std::cout << "minimum_correlation_frame" << minimum_correlation_frame << std::endl;
        double min_value = std::numeric_limits<double>::max();
        int32_t number_of_data = particial_confidence.cast<int>().array().sum();
        for (double sub_frame = -2.0; sub_frame <= 2.0; sub_frame += 0.0001)
        {
            Eigen::MatrixXd measured_angular_velocity_resampled = measured_angular_velocity->getResampledData(std::make_shared<ResamplerParameter>(frequency, (sub_frame + minimum_correlation_frame) , particial_estimated_angular_velocity.rows()));
            assert(measured_angular_velocity_resampled.rows() == particial_estimated_angular_velocity.rows());
            double value = ((measured_angular_velocity_resampled - particial_estimated_angular_velocity).array().colwise() * particial_confidence.array()).abs().sum() / (double)number_of_data;
            if (min_value > value)
            {
                min_value = value;
                minimum_correlation_subframe = sub_frame;
            }
        }
        std::cout << "min_value:" << min_value << std::endl;
        std::cout << "minimum_correlation_subframe:" << minimum_correlation_subframe;
    }
    minimum_correlation_subframe = (double)minimum_correlation_frame + minimum_correlation_subframe;

    return minimum_correlation_subframe;
}

// void VirtualGimbalManager::setResamplerParameter(double start, double new_frequency)
// {
//     if (0.0 == new_frequency)
//     {
//         resampler_parameter_ = std::make_shared<ResamplerParameter>(video_param->getFrequency(), start, estimated_angular_velocity->data.rows() * estimated_angular_velocity->getInterval());
//     }
//     else
//     {
//         resampler_parameter_ = std::make_shared<ResamplerParameter>(new_frequency, start, estimated_angular_velocity->data.rows() * estimated_angular_velocity->getInterval());
//     }
// }

// void VirtualGimbalManager::setResamplerParameter(ResamplerParameterPtr param)
// {
//     resampler_parameter_ = param;
// }

// Eigen::MatrixXd VirtualGimbalManager::getSynchronizedMeasuredAngularVelocity()
// {
//     Eigen::MatrixXd data;
//     data.resize(estimated_angular_velocity->data.rows(), estimated_angular_velocity->data.cols() + measured_angular_velocity->data.cols());
//     data.block(0, 0, estimated_angular_velocity->data.rows(), estimated_angular_velocity->data.cols()) = estimated_angular_velocity->data;
//     Eigen::MatrixXd resampled = measured_angular_velocity->getResampledData(resampler_parameter_);
//     assert(estimated_angular_velocity->data.rows() == resampled.rows());
//     data.block(0, estimated_angular_velocity->data.cols(), resampled.rows(), resampled.cols()) = resampled;
//     return data;
// }

// Eigen::MatrixXd VirtualGimbalManager::getRotationQuaternions()
// {
//     Eigen::MatrixXd data;
//     data.resize(estimated_angular_velocity->data.rows(), 4);
//     rotation_quaternion = std::make_shared<RotationQuaternion>(measured_angular_velocity, *resampler_parameter_);
//     for (int i = 0, e = data.rows(); i < e; ++i)
//     {

//         Eigen::MatrixXd temp = rotation_quaternion->getRotationQuaternion((double)i * video_param->getInterval()).coeffs().transpose();

//         data.row(i) = temp;
//     }
//     return data;
// }

std::map<int, std::vector<cv::Point2d>> VirtualGimbalManager::getCornerDictionary(cv::Size &pattern_size, bool debug_speedup, bool Verbose)
{
    auto capture = std::make_shared<cv::VideoCapture>(video_param->video_file_name); //動画をオープン
    std::map<int, std::vector<cv::Point2f>> corner_dict;
    cv::Mat gray_image;
    cv::Mat color_image;

    {
        std::vector<cv::Point2f> acquired_image_points;
        cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 20, 0.001);

        for (int i = 0, e = capture->get(cv::CAP_PROP_FRAME_COUNT); i < e; ++i)
        {
            (*capture) >> color_image;
            cv::cvtColor(color_image, gray_image, cv::COLOR_RGB2GRAY);
            if (cv::findChessboardCorners(gray_image, pattern_size, acquired_image_points, cv::CALIB_CB_FAST_CHECK))
            {
                cv::cornerSubPix(gray_image, acquired_image_points, cv::Size(11, 11), cv::Size(-1, -1), criteria);
                corner_dict[i] = acquired_image_points;
            }
            if (Verbose)
            {
                printf("%d/%d\r", i, e);
                std::cout << std::flush;
            }
            // Speed up for debug
            if (debug_speedup)
            {
                if (i == 100)
                    break;
            }
        }
    }

    std::map<int, std::vector<cv::Point2d>> retval;
    // return corner_dict;
    for (const auto &el : corner_dict)
    {
        for (const auto &el2 : el.second)
        {
            retval[el.first].push_back(cv::Point2d(el2.x, el2.y));
        }
    }
    return retval;
}

PointPairs VirtualGimbalManager::getFeaturePointsPairs()
{
    return  getFeaturePointsPairsFromVideo(video_param->video_file_name.c_str(), video_param->video_frames);
}

void VirtualGimbalManager::estimateAngularVelocity(const PointPairs &point_pairs, Eigen::MatrixXd &estimated_angular_velocity, Eigen::MatrixXd &confidence)
{
    Eigen::MatrixXd translation_and_rotation;

    convertFeaturePointsPairsToImageTranslationAndRotation(point_pairs, translation_and_rotation, confidence);
        estimated_angular_velocity.resize(translation_and_rotation.rows(), translation_and_rotation.cols());
    estimated_angular_velocity.col(0) =
        translation_and_rotation.col(1).unaryExpr([&](double a)
                                      { return video_param->getFrequency() * atan(a / (video_param->camera_info->fy_)); });
    estimated_angular_velocity.col(1) =
        translation_and_rotation.col(0).unaryExpr([&](double a)
                                      { return video_param->getFrequency() * -atan(a / (video_param->camera_info->fx_)); });
    estimated_angular_velocity.col(2) = -video_param->getFrequency() * translation_and_rotation.col(2);
}

void VirtualGimbalManager::getAngularVelocityFromJson(Eigen::MatrixXd &estimated_angular_velocity, Eigen::MatrixXd &confidence)
{
    Eigen::MatrixXd optical_flow;
    if (jsonExists(video_param->video_file_name))
    {
        readOpticalFlowFromJson(video_param->video_file_name, optical_flow, confidence);
    }
    else
    {
        std::cerr << "Failed to read Json file" << std::endl;
    }
    
    estimated_angular_velocity.resize(optical_flow.rows(), optical_flow.cols());
    estimated_angular_velocity.col(0) =
        optical_flow.col(1).unaryExpr([&](double a)
                                      { return video_param->getFrequency() * atan(a / (video_param->camera_info->fy_)); });
    estimated_angular_velocity.col(1) =
        optical_flow.col(0).unaryExpr([&](double a)
                                      { return video_param->getFrequency() * -atan(a / (video_param->camera_info->fx_)); });
    estimated_angular_velocity.col(2) = -video_param->getFrequency() * optical_flow.col(2);
}
/**
 * @brief Estimate angular velocity from video optical flow
 **/
// void VirtualGimbalManager::estimateAngularVelocity(Eigen::MatrixXd &estimated_angular_velocity, Eigen::MatrixXd &confidence)
// {
//     Eigen::MatrixXd optical_flow;
//     if (jsonExists(video_param->video_file_name))
//     {
//         readOpticalFlowFromJson(video_param->video_file_name, optical_flow, confidence);
//     }
//     else
//     {
//         CalcShiftFromVideo(video_param->video_file_name.c_str(), video_param->video_frames, optical_flow, confidence);
//     }
//     estimated_angular_velocity.resize(optical_flow.rows(), optical_flow.cols());
//     estimated_angular_velocity.col(0) =
//         optical_flow.col(1).unaryExpr([&](double a)
//                                       { return video_param->getFrequency() * atan(a / (video_param->camera_info->fy_)); });
//     estimated_angular_velocity.col(1) =
//         optical_flow.col(0).unaryExpr([&](double a)
//                                       { return video_param->getFrequency() * -atan(a / (video_param->camera_info->fx_)); });
//     estimated_angular_velocity.col(2) = -video_param->getFrequency() * optical_flow.col(2);
// }

// void VirtualGimbalManager::getUndistortUnrollingChessBoardPoints(double time_offset, const std::pair<int, std::vector<cv::Point2d>> &corner_dict, std::vector<cv::Point2d> &dst, double line_delay)
// {
//     getUndistortUnrollingChessBoardPoints(corner_dict.first * video_param->getInterval() + time_offset, corner_dict.second, dst, line_delay);
// }

/**
 * @brief Undistort and unrolling chess board board points. 
 **/
// void VirtualGimbalManager::getUndistortUnrollingChessBoardPoints(double time, const std::vector<cv::Point2d> &src, std::vector<cv::Point2d> &dst, double line_delay)
// {

//     // Collect time difference between video frame and gyro frame. These frame rates are deferent, so that time should be compensated.
//     time += (measured_angular_velocity->getInterval() - estimated_angular_velocity->getInterval()) * 0.5;
//     //手順
//     //1.補正前画像を分割した時の分割点の座標(pixel)を計算
//     //2.1の座標を入力として、各行毎のW(t1,t2)を計算
//     //3.補正後の画像上のポリゴン座標(pixel)を計算、歪み補正も含める

//     for (const auto &el : src) //(int j = 0; j <= division_y; ++j)
//     {
//         //W(t1,t2)を計算
//         //1
//         double v = el.y;

//         double time_in_row = line_delay * (v - video_param->camera_info->height_ * 0.5);
//         Eigen::MatrixXd R = (rotation_quaternion->getRotationQuaternion(time_in_row + time).conjugate() * rotation_quaternion->getRotationQuaternion(time)).matrix();
//         {
//             double u = el.x;
//             //後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
//             Eigen::Vector3d p;
//             p << (u - video_param->camera_info->cx_) / video_param->camera_info->fx_, (v - video_param->camera_info->cy_) / video_param->camera_info->fy_, 1.0; // Homogenious coordinate
//             //2
//             Eigen::MatrixXd XYW = R * p;

//             double x1 = XYW(0, 0) / XYW(2, 0);
//             double y1 = XYW(1, 0) / XYW(2, 0);

//             double r = sqrt(x1 * x1 + y1 * y1);

//             double x2 = x1 * (1.0 + video_param->camera_info->inverse_k1_ * r * r + video_param->camera_info->inverse_k2_ * r * r * r * r) + 2.0 * video_param->camera_info->inverse_p1_ * x1 * y1 + video_param->camera_info->inverse_p2_ * (r * r + 2.0 * x1 * x1);
//             double y2 = y1 * (1.0 + video_param->camera_info->inverse_k1_ * r * r + video_param->camera_info->inverse_k2_ * r * r * r * r) + video_param->camera_info->inverse_p1_ * (r * r + 2.0 * y1 * y1) + 2.0 * video_param->camera_info->inverse_p2_ * x1 * y1;
//             //変な折り返しを防止
//             if ((pow(x2 - x1, 2) > 1.0) || (pow(y2 - y1, 2) > 1.0))
//             {
//                 //                printf("折り返し防止\r\n");
//                 x2 = x1;
//                 y2 = y1;
//             }
//             dst.push_back(cv::Point2d(
//                 x2 * video_param->camera_info->fx_ + video_param->camera_info->cx_,
//                 y2 * video_param->camera_info->fy_ + video_param->camera_info->cy_));
//         }
//     }
//     return;
// }

double VirtualGimbalManager::computeReprojectionErrors(const vector<vector<Point3d>> &objectPoints,
                                                       const vector<vector<Point2d>> &imagePoints,
                                                       const vector<Mat> &rvecs, const vector<Mat> &tvecs,
                                                       const Mat &cameraMatrix, const Mat &distCoeffs,
                                                       vector<double> &residuals, bool fisheye)
{
    vector<Point2d> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    Point2d diff;

    residuals.resize(imagePoints.size() * imagePoints.begin()->size() * 2);
    auto residuals_itr = residuals.begin();

    for (size_t i = 0; i < objectPoints.size(); ++i)
    {
        if (fisheye)
        {
            fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
                                   distCoeffs);
        }
        else
        {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        }
        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        for (size_t k = 0; k < imagePoints2.size(); ++k)
        {
            diff = imagePoints[i][k] - imagePoints2[k];
            *(residuals_itr++) = diff.x;
            *(residuals_itr++) = diff.y;
        }
        size_t n = objectPoints[i].size();

        totalErr += err * err;
        totalPoints += n;
    }

    return std::sqrt(totalErr / totalPoints);
}

void VirtualGimbalManager::setFilter(FilterPtr filter)
{
    filter_ = filter;
}

void VirtualGimbalManager::setMaximumGradient(double value)
{
    maximum_gradient_ = value;
}

Eigen::VectorXd VirtualGimbalManager::getFilterCoefficients(double zoom,
                                                            FilterPtr filter,
                                                            std::vector<std::pair<int32_t, double>> &sync_table,
                                                            int32_t strongest_filter_param, int32_t weakest_filter_param)
{

    Eigen::VectorXd filter_strength(video_param->video_frames);
    //Calcurate in all frame
    for (int frame = 0, e = filter_strength.rows(); frame < e; ++frame)
    {

        // フィルタが弱くて、簡単な条件で、黒帯が出るなら、しょうが無いからこれを採用
        if (hasBlackSpace(frame, zoom, measured_angular_velocity, video_param, filter->getFilterCoefficient(weakest_filter_param), sync_table))
        {
            filter_strength[frame] = weakest_filter_param;
        }
        // フィルタが強くて、すごく安定化された条件で、難しい条件で、黒帯が出ないなら、喜んでこれを採用
        else if (!hasBlackSpace(frame, zoom, measured_angular_velocity, video_param, filter->getFilterCoefficient(strongest_filter_param), sync_table))
        {
            filter_strength[frame] = strongest_filter_param;
        }
        else
        {
            filter_strength[frame] = bisectionMethod(frame, zoom, measured_angular_velocity, video_param, filter, sync_table, strongest_filter_param, weakest_filter_param);
        }
    }
    //    std::cout << filter_strength << std::endl;
    gradientLimit(filter_strength, maximum_gradient_);

    return (filter_strength);
}

std::shared_ptr<cv::VideoCapture> VirtualGimbalManager::getVideoCapture()
{
    return std::make_shared<cv::VideoCapture>(video_param->video_file_name);
}

#define LAP_BEGIN //auto td1=std::chrono::system_clock::now();int line =__LINE__;
#define LAP       // printf("\r\nDuration from L %d to %d is %ld\r\n", line,__LINE__, std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - td1).count());line=__LINE__;td1=std::chrono::system_clock::now();

int VirtualGimbalManager::spin(double zoom, FilterPtr filter, Eigen::VectorXd &filter_strength, std::vector<std::pair<int32_t, double>> &sync_table, bool show_image)
{

    // Prepare OpenCL
    cv::ocl::Context context;
    initializeCL(context);

    // Open Video
    reader_ = std::make_shared<MultiThreadVideoReader>(video_param->video_file_name, queue_size_);

    // Prepare
    measured_angular_velocity->calculateAngleQuaternion();

    UMatPtr umat_p_latest;

    constexpr bool OPENCL_SUCCESS = true;

    if (show_image)
    {
        cv::namedWindow("Original", cv::WINDOW_NORMAL);
        cv::namedWindow("Result", cv::WINDOW_NORMAL);
    }

    for (int frame = 0; frame < video_param->video_frames; ++frame)
    {
        UMatPtr umat_p_src;
        reader_->get(umat_p_src);
        if (!umat_p_latest)
        {
            umat_p_latest = UMatPtr(new cv::UMat(umat_p_src->size(), CV_8UC4, cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY));
        }

        Eigen::Quaterniond stabilized_angle_quaternion;
        measured_angular_velocity->getStabilizedQuaternion(frame, filter->getFilterCoefficient(filter_strength(frame)), sync_table, stabilized_angle_quaternion);

        umat_p_latest->setTo(cv::Scalar(127, 127, 127, 255)); // BGR A, A channel means distance from target frame
        std::vector<float> stabilized_angle_matrices;
        measured_angular_velocity->getCorrectionMatrices(stabilized_angle_quaternion, frame, video_param->camera_info->height_, video_param->camera_info->line_delay_ * video_param->getFrequency(), sync_table, stabilized_angle_matrices);
        assert(frame >= frame);
        int distance = frame - frame;
        if (OPENCL_SUCCESS != fillPixelValues(context, zoom, stabilized_angle_matrices, distance, umat_p_src, umat_p_latest))
        {
            std::cerr << "Failed to run a fill_function kernel at Line:" << __LINE__ << " of " << __FILE__ << std::endl;
            return -1;
        }

        // Show image on display
        if (show_image)
        {
            cv::UMat small, small_src;
            cv::resize(*umat_p_latest, small, cv::Size(), 0.5, 0.5);
            cv::resize(*umat_p_src, small_src, cv::Size(), 0.5, 0.5);
            cv::imshow("Original", small_src);
            cv::imshow("Result", small);
            char key = cv::waitKey(1);
            if ('q' == key)
            {
                break;
            }
            else if ('s' == key)
            {
                sleep(1);
                key = cv::waitKey(0);
                if ('q' == key)
                {
                    cv::destroyAllWindows();
                    return 0;
                }
            }
        }

        if (writer_)
        {
            UMatPtr copied(new cv::UMat(umat_p_latest->clone()));
            writer_->push(copied);
        }

        {
            //Show fps
            auto t4 = std::chrono::system_clock::now();
            static auto t3 = t4;
            // 処理の経過時間
            double elapsedmicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();
            static double fps = 0.0;
            if (elapsedmicroseconds != 0.0)
            {
                fps = 0.05 * (1e6 / elapsedmicroseconds) + 0.95 * fps;
            }
            t3 = t4;
            printf("fps:%4.2f\r", fps);
            fflush(stdout);
        }
    }
    cv::destroyAllWindows();
    return 0;
}

cv::Point2f VirtualGimbalManager::warp_undistort(const cv::Point2f &p, float zoom_ratio, const std::vector<float> &stabilized_angle_matrices)
{
    // prepare intrinsic parameters
    float k1 = video_param->camera_info->inverse_k1_;
    float k2 = video_param->camera_info->inverse_k2_;
    float p1 = video_param->camera_info->inverse_p1_;
    float p2 = video_param->camera_info->inverse_p2_;
    float fx = video_param->camera_info->fx_;
    float fy = video_param->camera_info->fy_;
    float cx = video_param->camera_info->cx_;
    float cy = video_param->camera_info->cy_;

    cv::Point2f c(cx,cy);
    cv::Point2f f(fx,fy);
    // cv::Point2f x1 = (p-c)/f;// (float2)((u - cx)/fx,(v-cy)/fy,1.f);
    cv::Point2f x1((p.x - c.x)/f.x,(p.y-c.y)/f.y);
    float r2 = x1.dot(x1);
    cv::Point2f x2 = x1*(1.f + k1*r2+k2*r2*r2);
    x2 += cv::Point2f(2.f*p1*x1.x*x1.y+p2*(r2+2.f*x1.x*x1.x), p1*(r2+2.f*x1.y*x1.y)+2.f*p2*x1.x*x1.y);
    //折り返しの話はとりあえずスキップ

    cv::Point3f x3 = cv::Point3f(x2.x,x2.y,1.0);
    auto R = stabilized_angle_matrices.begin() + 9 * std::round(p.y);
    cv::Point3f XYZ = cv::Point3f(R[0] * x3.x + R[1] * x3.y + R[2] * x3.z,
                        R[3] * x3.x + R[4] * x3.y + R[5] * x3.z,
                        R[6] * x3.x + R[7] * x3.y + R[8] * x3.z);
    x2 = cv::Point2f(XYZ.x,XYZ.y) / XYZ.z;
    return cv::Point2f(x2.x * f.x * zoom_ratio + c.x, x2.y * f.y * zoom_ratio + c.y);
}

int VirtualGimbalManager::spinAnalyse(double zoom, FilterPtr filter,Eigen::VectorXd &filter_strength, std::vector<std::pair<int32_t,double>> &sync_table, const PointPairs &point_pairs, char* experimental_param_json_path)
{

    // Modify sync table
    std::string path_prefix;
    if(experimental_param_json_path)
    {
        struct stat st;
        if(stat(experimental_param_json_path,&st)){
            printf("%s is not found.",experimental_param_json_path);
            return -1;
        }
        FILE* fp = fopen(experimental_param_json_path, "rb"); // non-Windows use "r"
        std::vector<char> readBuffer((intmax_t)st.st_size+10);
        rapidjson::FileReadStream is(fp, readBuffer.data(), readBuffer.size());
        rapidjson::Document e;
        e.ParseStream(is);
        fclose(fp);

        // return e["frame_offset"].GetDouble();

        // Modify sync table
        // Sync table is coefficients of linear expression, y = ax + b, between measured and estimated angular velocity.
        // Unit is frame, not second.
        for(auto &el:sync_table)
        {
            // Add frame offset
            el.second += e["frame_offset"].GetDouble();
        }

        path_prefix = e["path_prefix"].GetString();
    }

    PointPairs warped_point_paires;
            
    // Prepare
    measured_angular_velocity->calculateAngleQuaternion();

    std::vector<float> stabilized_angle_matrices_prev;

    for(size_t frame = 0; frame<point_pairs.size(); ++frame)
    {
        // Get stabilization
        std::vector<float> stabilized_angle_matrices_curr;
        
        {
            Eigen::Quaterniond stabilized_angle_quaternion;
            measured_angular_velocity->getStabilizedQuaternion(frame+1, filter->getFilterCoefficient(filter_strength(frame+1)), sync_table, stabilized_angle_quaternion);
            // Get rotation matrix in each line of the frame
            measured_angular_velocity->getCorrectionMatrices(stabilized_angle_quaternion, frame+1, video_param->camera_info->height_, video_param->camera_info->line_delay_ * video_param->getFrequency(), sync_table, stabilized_angle_matrices_curr);
        }

        // Get previous one at once.
        if(stabilized_angle_matrices_prev.empty())
        {
            Eigen::Quaterniond stabilized_angle_quaternion_prev;
            measured_angular_velocity->getStabilizedQuaternion(frame, filter->getFilterCoefficient(filter_strength(frame)), sync_table, stabilized_angle_quaternion_prev);
            measured_angular_velocity->getCorrectionMatrices(stabilized_angle_quaternion_prev, frame, video_param->camera_info->height_, video_param->camera_info->line_delay_ * video_param->getFrequency(), sync_table, stabilized_angle_matrices_prev);
        }

        // Warp point
        std::vector<cv::Point2f> warped_points_prev;
        std::vector<cv::Point2f> warped_points_curr;    
        for(const auto &point:point_pairs[frame].first) // previous frame
        {
            warped_points_prev.push_back(warp_undistort(point,  zoom, stabilized_angle_matrices_prev));
        }
        
        for(const auto &point:point_pairs[frame].second) // 
        {
            warped_points_curr.push_back(warp_undistort(point, zoom, stabilized_angle_matrices_curr));
        }   
        
        warped_point_paires.push_back(PointPair(warped_points_prev,warped_points_curr));
        
        // Save it to prepare next frame
        stabilized_angle_matrices_prev = stabilized_angle_matrices_curr;
    }
    
    

    
    // Calculate angular velocity
    Eigen::MatrixXd warped_estimated_angular_velocity,warped_confidence;

    estimateAngularVelocity(warped_point_paires,warped_estimated_angular_velocity,warped_confidence);
    

    if(warped_estimated_angular_velocity.rows()){
        LoggingDouble d;
        for(int r=0;r<warped_estimated_angular_velocity.rows();++r)
        {
            d["Frame"].push_back((double)r);
            d["rx"].push_back(warped_estimated_angular_velocity(r,0));
            d["ry"].push_back(warped_estimated_angular_velocity(r,1));
            d["rz"].push_back(warped_estimated_angular_velocity(r,2));
        }
        std::string time_stamp = DataCollection::getSystemTimeStamp();
        DataCollection collection("timestamp_"+time_stamp + "_warped_estimated_angular_velocity.csv");

        if(path_prefix.empty())
        {
            collection.setDuplicateFilePath("latest_warped_estimated_angular_velocity.csv");
        }
        else
        {
            collection.setDuplicateFilePath(path_prefix + "_warped_estimated_angular_velocity.csv");
        }

        collection.set(d);
    }
    else
    {
        std::cout << "warped_estimated_angular_velocity is empty." << std::endl;
    }

    return 0;
}

#undef LAP_BEGIN
#undef LAP

int VirtualGimbalManager::spinInpainting(double zoom, std::vector<std::pair<int32_t, double>> &sync_table, FilterPtr filter, size_t buffer_size, int filter_strength, bool show_image)
{
    // Prepare OpenCL
    cv::ocl::Context context;
    initializeCL(context);

    // Open Video
    reader_ = std::make_shared<MultiThreadVideoReader>(video_param->video_file_name, queue_size_);

    // Prepare
    measured_angular_velocity->calculateAngleQuaternion();

    // UMatの準備
    // UMatのバッファ
    UMatMap b;

    // size_t buffer_size = 3;//21
    for (size_t i = 0; i < buffer_size / 2; ++i)
    {
        reader_->get(b[i]);
        if (!b[i])
        {
            std::cerr << "Failed to read video frame" << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }

    // BGRAのバッファ2枚 (b_past,b_future)
    UMatPtr b_past(new cv::UMat(b[0]->size(), CV_8UC4, cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY));
    UMatPtr b_future(new cv::UMat(b[0]->size(), CV_8UC4, cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY));

    // 出力画像のBGRAの1枚 (b_output)
    UMatPtr b_output(new cv::UMat(b[0]->size(), CV_8UC4, cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY));

    constexpr bool OPENCL_SUCCESS = true;

    if (show_image)
    {
        cv::namedWindow("Original", cv::WINDOW_NORMAL);
        cv::namedWindow("Result", cv::WINDOW_NORMAL);
        cv::namedWindow("inpaint", cv::WINDOW_NORMAL);
    }

    // TODO: MultiThreadVideoReaderがすでに内部にbufferをもっているので、こいつを借りてもいい気がする
    for (int frame = 0; frame < video_param->video_frames; ++frame)
    {
        // 動画最も古い1フレームを削除
        while (b.size() > buffer_size)
        {
            b.erase(b.begin());
        }
        // 動画最新1フレーム読み込み
        int back_index = frame + buffer_size / 2;
        UMatPtr p;
        reader_->get(p);
        if (p)
        {
            b[back_index] = std::move(p);
        }

        // フィルタ済み姿勢計算 @ 注目フレームの基準時間
        // getRelativeAngleで前後のフレームは得られるらしい
        // Privateなので直接アクセスできないorz
        // 以下の関数で、補正用のquaternionと、相対角度のquaternionを、estimatedな時間で取得する
        // std::vector<Eigen::Quaterniond> measured_angle_quaternions(video_param->camera_info->height_*buffer_size);
        Eigen::Quaterniond stabilized_angle_quaternion;

        double diff_angle = measured_angular_velocity->getStabilizedQuaternion(frame, filter->getFilterCoefficient(filter_strength), sync_table, stabilized_angle_quaternion);

        // UMatとしてつくる　col は 時間軸(フレーム、行数)　rowはバッファーのサイズ(=フレーム数)に対応
        // static cv::UMat correction_matrices(cv::Size(video_param->camera_info->height_ * 9, buffer_size),CV_32F, cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY);

        std::cout << "angle:" << diff_angle << std::endl;

        // b_futureについて基準フレームから未来方向へfor文で連続して画素を埋めていく
        b_future->setTo(cv::Scalar(127, 127, 127, 255)); // BGR A, A channel means distance from target frame
        for (int future_frame = frame + buffer_size / 2; frame <= future_frame; --future_frame)
        {
            if (b.count(future_frame) == 0)
            {
                continue;
            }
            std::vector<float> stabilized_angle_matrices;
            measured_angular_velocity->getCorrectionMatrices(stabilized_angle_quaternion, future_frame, video_param->camera_info->height_, video_param->camera_info->line_delay_ * video_param->getFrequency(), sync_table, stabilized_angle_matrices);
            assert(future_frame >= frame);
            int distance = future_frame - frame;
            if (OPENCL_SUCCESS != fillPixelValues(context, zoom, stabilized_angle_matrices, distance, b[future_frame], b_future))
            {
                std::cerr << "Failed to run a fill_function kernel at Line:" << __LINE__ << " of " << __FILE__ << std::endl;
                return -1;
            }
        }

        // b_pastについて基準フレームから過去方向へfor文で連続して画素を埋めていく
        b_past->setTo(cv::Scalar(127, 127, 127, 255));
        for (int past_frame = frame - buffer_size / 2; past_frame <= frame; ++past_frame)
        {
            if (b.count(past_frame) == 0)
            {
                continue;
            }
            std::vector<float> stabilized_angle_matrices;
            measured_angular_velocity->getCorrectionMatrices(stabilized_angle_quaternion, past_frame, video_param->camera_info->height_, video_param->camera_info->line_delay_ * video_param->getFrequency(), sync_table, stabilized_angle_matrices);
            assert(frame >= past_frame);
            int distance = frame - past_frame;
            if (OPENCL_SUCCESS != fillPixelValues(context, zoom, stabilized_angle_matrices, distance, b[past_frame], b_past))
            {
                std::cerr << "Failed to run a fill_function kernel at Line:" << __LINE__ << " of " << __FILE__ << std::endl;
                return -1;
            }
        }

        // 最後に b_pastとb_futureからb_outputを生成
        if (OPENCL_SUCCESS != interpolatePixels(context, b_past, b_future, b_output))
        {
            std::cerr << "Failed to run a interpoloate_function kernel at Line:" << __LINE__ << " of " << __FILE__ << std::endl;
            return -1;
        }

        // Inpaint test
        if (frame + 1 < video_param->video_frames)
        {
            static UMatPtr b_latest;
            if (!b_latest)
                b_latest = UMatPtr(new cv::UMat(b[0]->size(), CV_8UC4, cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY));
            static UMatPtr b_next;
            if (!b_next)
                b_next = UMatPtr(new cv::UMat(b[0]->size(), CV_8UC4, cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY));

            // Generate latest frame
            std::vector<float> stabilized_angle_matrices;
            measured_angular_velocity->getCorrectionMatrices(stabilized_angle_quaternion, frame, video_param->camera_info->height_, video_param->camera_info->line_delay_ * video_param->getFrequency(), sync_table, stabilized_angle_matrices);
            if (OPENCL_SUCCESS != fillPixelValues(context, zoom, stabilized_angle_matrices, 0, b[frame], b_latest))
            {
                std::cerr << "Failed to run a fill_function kernel at Line:" << __LINE__ << " of " << __FILE__ << std::endl;
                return -1;
            }

            // Generate next frame
            stabilized_angle_matrices.clear();
            measured_angular_velocity->getCorrectionMatrices(stabilized_angle_quaternion, frame + 1, video_param->camera_info->height_, video_param->camera_info->line_delay_ * video_param->getFrequency(), sync_table, stabilized_angle_matrices);
            if (OPENCL_SUCCESS != fillPixelValues(context, zoom, stabilized_angle_matrices, 0, b[frame + 1], b_next))
            {
                std::cerr << "Failed to run a fill_function kernel at Line:" << __LINE__ << " of " << __FILE__ << std::endl;
                return -1;
            }

            cv::UMat mono_latest;
            cv::UMat mono_next;
            cv::cvtColor(*b_latest, mono_latest, COLOR_BGRA2GRAY);
            cv::cvtColor(*b_next, mono_next, COLOR_BGRA2GRAY);
            cv::Mat float_latest, float_next;
            mono_latest.convertTo(float_latest, CV_32F);
            mono_next.convertTo(float_next, CV_32F);
            cv::Size map_size = cv::Size(5, 5);
            cv::Size window_size = cv::Size(600, 400);
            cv::Mat map = calculateOpticalFlow(map_size, window_size, float_latest, float_next);

            //  = b_next->getMat(cv::ACCESS_READ)
            cv::Mat latest = b_latest->getMat(cv::ACCESS_READ).clone();
            visualizeInpaintingMap(latest, window_size, map);

            if (writer_)
            {
                UMatPtr copied(new cv::UMat(latest.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY)));
                writer_->push(copied);
            }

            cv::resize(latest, latest, cv::Size(), 0.3, 0.3);
            cv::imshow("inpaint", latest);
        }

        // Show image on displayreturn 0;
        if (show_image)
        {
            // cv::imshow("Original", *b[frame]);
            // cv::imshow("Result", *b_output);
            cv::UMat small, small_src;
            cv::resize(*b_output, small, cv::Size(), 0.5, 0.5);
            cv::resize(*b[frame], small_src, cv::Size(), 0.5, 0.5);
            cv::imshow("Original", small_src);
            cv::imshow("Result", small);
            char key = cv::waitKey(1);
            if ('q' == key)
            {
                break;
            }
            else if ('s' == key)
            {
                sleep(1);
                key = cv::waitKey(0);
                if ('q' == key)
                {
                    cv::destroyAllWindows();
                    return 0;
                }
            }
        }

        if (writer_)
        {
            UMatPtr copied(new cv::UMat(b_output->clone()));
            writer_->push(copied);
        }

        {
            //Show fps
            auto t4 = std::chrono::system_clock::now();
            static auto t3 = t4;
            // 処理の経過時間
            double elapsedmicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();
            static double fps = 0.0;
            if (elapsedmicroseconds != 0.0)
            {
                fps = 0.05 * (1e6 / elapsedmicroseconds) + 0.95 * fps;
            }
            t3 = t4;
            printf("fps:%4.2f\r", fps);
            fflush(stdout);
        }
    }
    cv::destroyAllWindows();
    return 0;
}

bool VirtualGimbalManager::fillPixelValues(cv::ocl::Context &context, double zoom, std::vector<float> stabilized_angle_matrices, uint8_t distance, UMatPtr &source_image, UMatPtr &dest_image)
{
    // Prepare OpenCL
    // cv::ocl::Context context;
    // cv::Mat mat_src = cv::Mat::zeros(video_param->camera_info->height_, video_param->camera_info->width_, CV_8UC4); // TODO:冗長なので書き換える
    // cv::UMat umat_src = mat_src.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);

    cv::Mat mat_matrices = cv::Mat(stabilized_angle_matrices.size(), 1, CV_32F, stabilized_angle_matrices.data());
    cv::UMat umat_matrices = mat_matrices.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);

    // Stabilize every frames
    float ik1 = video_param->camera_info->inverse_k1_;
    float ik2 = video_param->camera_info->inverse_k2_;
    float ip1 = video_param->camera_info->inverse_p1_;
    float ip2 = video_param->camera_info->inverse_p2_;
    float fx = video_param->camera_info->fx_;
    float fy = video_param->camera_info->fy_;
    float cx = video_param->camera_info->cx_;
    float cy = video_param->camera_info->cy_;
    cv::Mat mat_params = (cv::Mat_<float>(9, 1) << (float)zoom, ik1, ik2, ip1, ip2, fx, fy, cx, cy);
    cv::UMat umat_params = mat_params.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);

    cv::String build_opt;
    // initializeCL(context);

    cv::ocl::Image2D image(*source_image);

    cv::ocl::Kernel kernel;
    try
    {
        getKernel(kernel_name, "fill_function", kernel, context, build_opt);
    }
    catch (const char *e)
    {
        std::cerr << "Error: " << e << '\n';
        std::exit(EXIT_FAILURE);
    }

    kernel.args(image,
                cv::ocl::KernelArg::WriteOnly(*dest_image),
                cv::ocl::KernelArg::ReadOnlyNoSize(umat_matrices),
                cv::ocl::KernelArg::ReadOnlyNoSize(umat_params),
                distance);
    size_t globalThreads[3] = {(size_t)dest_image->cols, (size_t)dest_image->rows, 1};
    bool success = kernel.run(3, globalThreads, NULL, true);
    return success;
}

bool VirtualGimbalManager::interpolatePixels(cv::ocl::Context &context, UMatPtr &past, UMatPtr &future, UMatPtr &output)
{
    // Prepare OpenCL
    // cv::ocl::Context context;

    cv::String build_opt;
    // initializeCL(context);

    cv::ocl::Image2D past_image(*past);
    cv::ocl::Image2D future_image(*future);
    cv::ocl::Kernel kernel;
    try
    {
        getKernel(kernel_name, "interpoloate_function", kernel, context, build_opt);
    }
    catch (const char *e)
    {
        std::cerr << "Error: " << e << '\n';
        std::exit(EXIT_FAILURE);
    }

    kernel.args(past_image,
                future_image,
                cv::ocl::KernelArg::WriteOnly(*output));
    size_t globalThreads[3] = {(size_t)output->cols, (size_t)output->rows, 1};
    bool success = kernel.run(3, globalThreads, NULL, true);
    return success;
}

void VirtualGimbalManager::enableWriter(const char *video_path)
{
    writer_ = std::make_shared<MultiThreadVideoWriter>(MultiThreadVideoWriter::getOutputName(video_path), *video_param, queue_size_);
}

std::vector<std::pair<int32_t, double>> VirtualGimbalManager::getSyncTable(double period_in_second, int32_t width)
{
    assert(width % 2);          // Odd
    int32_t radius = width / 2; // radius and width means number of frame in estimated angular velocity, not measured angular velocity.
    std::vector<std::pair<int32_t, double>> table;
    

    double offset_frame_between_optical_flow_and_image = 0.5;
    for (int estimated_frame = radius, e = estimated_angular_velocity->getFrames() - radius; estimated_frame < e; estimated_frame += (int32_t)(period_in_second * video_param->getFrequency()))
    {
        table.emplace_back(estimated_frame + offset_frame_between_optical_flow_and_image,getMeasuredFramePositionFrom(estimated_frame,width));
    }
    return table;
}

PointPairs VirtualGimbalManager::getWarpedPointPairs(double zoom, FilterPtr filter,Eigen::VectorXd &filter_strength, const PointPairs &point_pairs, int32_t start_frame, int32_t frame_length, std::vector<std::pair<int32_t,double>> &sync_table)
{
    PointPairs warped_point_pairs;
            
    // Prepare
    // measured_angular_velocity->calculateAngleQuaternion();

    std::vector<float> stabilized_angle_matrices_prev;

    for(int32_t frame = start_frame; frame< start_frame + frame_length; ++frame)
    {
        // Get stabilization
        std::vector<float> stabilized_angle_matrices_curr;
        
        {
            Eigen::Quaterniond stabilized_angle_quaternion;
            measured_angular_velocity->getStabilizedQuaternion(frame+1, filter->getFilterCoefficient(filter_strength(frame)), sync_table, stabilized_angle_quaternion);
            // Get rotation matrix in each line of the frame
            measured_angular_velocity->getCorrectionMatrices(stabilized_angle_quaternion, frame+1, video_param->camera_info->height_, video_param->camera_info->line_delay_ * video_param->getFrequency(), sync_table, stabilized_angle_matrices_curr);
        }

        // Get previous one at once.
        if(stabilized_angle_matrices_prev.empty())
        {
            Eigen::Quaterniond stabilized_angle_quaternion_prev;
            measured_angular_velocity->getStabilizedQuaternion(frame, filter->getFilterCoefficient(filter_strength(frame)), sync_table, stabilized_angle_quaternion_prev);
            measured_angular_velocity->getCorrectionMatrices(stabilized_angle_quaternion_prev, frame, video_param->camera_info->height_, video_param->camera_info->line_delay_ * video_param->getFrequency(), sync_table, stabilized_angle_matrices_prev);
        }

        // Warp point
        std::vector<cv::Point2f> warped_points_prev;
        std::vector<cv::Point2f> warped_points_curr;    
        for(const auto &point:point_pairs[frame].first) // previous frame
        {
            warped_points_prev.push_back(warp_undistort(point,  zoom, stabilized_angle_matrices_prev));
        }
        
        for(const auto &point:point_pairs[frame].second) // 
        {
            warped_points_curr.push_back(warp_undistort(point, zoom, stabilized_angle_matrices_curr));
        }   
        
        warped_point_pairs.push_back(PointPair(warped_points_prev,warped_points_curr));
        
        // Save it to prepare next frame
        stabilized_angle_matrices_prev = stabilized_angle_matrices_curr;
    }

    return warped_point_pairs;
}

double VirtualGimbalManager::getAverageAbsoluteAngularAcceleration(const PointPairs &point_pairs, double frequency)
{
    // Get angular velocity
    Eigen::MatrixXd partial_estimated_angular_velocity;
    Eigen::MatrixXd partial_confidence;
    estimateAngularVelocity(point_pairs,partial_estimated_angular_velocity,partial_confidence);

    double absolute_angular_acceleration 
    = (( partial_estimated_angular_velocity.block(1,0,partial_estimated_angular_velocity.rows()-1,partial_estimated_angular_velocity.cols())
        - partial_estimated_angular_velocity.block(0,0,partial_estimated_angular_velocity.rows()-1,partial_estimated_angular_velocity.cols())
        ).array()).abs().mean();
    
    return absolute_angular_acceleration;
}

SyncTable VirtualGimbalManager::createSyncTable(int32_t estimated_frame, double measured_frame, double e2m_ratio)
{
    SyncTable table;
    int32_t start_estimated_frame = 0;
    int32_t end_estimated_frame = estimated_angular_velocity->getFrames()-1;
    table.emplace_back(start_estimated_frame,e2m_ratio*(start_estimated_frame-estimated_frame)+measured_frame);
    table.emplace_back(end_estimated_frame,e2m_ratio*(end_estimated_frame-estimated_frame)+measured_frame);
    return table;
}

std::vector<std::pair<int32_t, double>> VirtualGimbalManager::getSyncTable(double zoom, FilterPtr filter, int32_t filter_length, PointPairs &point_pairs, double sync_interval_sec, double ra4_length_sec)
{

    measured_angular_velocity->calculateAngleQuaternion();

    Eigen::VectorXd filter_strength = Eigen::VectorXd::Ones(video_param->video_frames).array() * filter_length;
    
    EstimatedFrame ra4_length_efs = ra4_length_sec * estimated_angular_velocity->getFrequency();
    assert(point_pairs.size()>(size_t)ra4_length_efs);

    std::vector<std::pair<int32_t, double>> table;
    if(sync_interval_sec + ra4_length_sec > estimated_angular_velocity->getLengthInSecond())
    {
        // TODO: Short video mode
        std::cout << "Implement short video mode." << std::endl;
        std::exit(EXIT_FAILURE);
    }
    else
    {
        // Normal video mode

        
        // ここで仮のSyncTableを作るための必要な定数を定義
        const double e2m = measured_angular_velocity->getFrequency() / estimated_angular_velocity->getFrequency();
        const MeasuredFrame m = measured_angular_velocity->data.rows();
        const EstimatedFrame e = estimated_angular_velocity->data.rows();
        const MeasuredFrame d_max = m - e2m * e;
        const EstimatedFrame duration_in_efs = (EstimatedFrame)(sync_interval_sec * estimated_angular_velocity->getFrequency());

        assert(d_max >= 0);


        // Head synchronization
        EstimatedFrame e_frame = 0;
        MeasuredFrame m_frame = (e_frame + 1 ) * e2m + filter_length;    // ローリングシャッター補正のために、 基準フレームよりも負の位置にアクセスする。そのため1フレーム分あらかじめオフセットしておいて、例外を防ぐ。

        {
            
            // ここで部分的なfeature point pairsを生成
            PointPairs particial_point_pairs;
            std::copy(point_pairs.begin()+e_frame,point_pairs.begin()+e_frame+ra4_length_efs,std::back_inserter(particial_point_pairs));

            // Get angular acceleration
            double raw_aaaa = getAverageAbsoluteAngularAcceleration(particial_point_pairs,estimated_angular_velocity->getFrequency());
            
            LoggingDouble ld;
            for(double d=0;d<d_max;d+=1)
            {
                SyncTable sync_table = createSyncTable(e_frame,m_frame+d,e2m); // TODO:何かの初期化関数
                constexpr EstimatedFrame start_efs = 0;
                PointPairs warped_point_pairs = getWarpedPointPairs(zoom, filter, filter_strength, particial_point_pairs, start_efs, particial_point_pairs.size(), sync_table);
                double warped_aaaa = getAverageAbsoluteAngularAcceleration(warped_point_pairs,estimated_angular_velocity->getFrequency());

                ld["Measured Frame"].push_back(m_frame+d);
                ld["Ratio"].push_back(warped_aaaa/raw_aaaa);

            }

            DataCollection collection("aaaa_ratio.csv");
            collection.set(ld);
        }
        
        // Middle synchronization

        // Tail synchronization
        e_frame = point_pairs.size() - 1 - ra4_length_efs;
        // m_frame = (e_frame - 1 - filter_length / 2.) * e2m ;    // ローリングシャッター補正のために、 基準フレームよりも負の位置にアクセスする。そのため1フレーム分あらかじめオフセットしておいて、例外を防ぐ。
        // m_frame = (e_frame -1 ) * e2m - filter_length / 2. ;    // ローリングシャッター補正のために、 基準フレームよりも負の位置にアクセスする。そのため1フレーム分あらかじめオフセットしておいて、例外を防ぐ。
        m_frame = (e_frame -1 ) * e2m - filter_length ;    // ローリングシャッター補正のために、 基準フレームよりも負の位置にアクセスする。そのため1フレーム分あらかじめオフセットしておいて、例外を防ぐ。
        {
            
            // ここで部分的なfeature point pairsを生成
            PointPairs particial_point_pairs;
            std::cout << "e_frame:" << e_frame << std::endl;
            std::cout << "m_frame:" << m_frame << std::endl;
            std::cout << "ra4_length_efs:" << ra4_length_efs << std::endl;
            std::cout << "point_pairs.size():" << point_pairs.size() << std::endl; 
            std::cout << "measured_angular_velocity->data.rows():" << measured_angular_velocity->data.rows() << std::endl;

            std::copy(point_pairs.begin()+e_frame,point_pairs.begin()+e_frame+ra4_length_efs,std::back_inserter(particial_point_pairs));

            // Get angular acceleration
            double raw_aaaa = getAverageAbsoluteAngularAcceleration(particial_point_pairs,estimated_angular_velocity->getFrequency());
            
            LoggingDouble ld;
            constexpr EstimatedFrame start_efs = 0;
            for(double d=0;d<d_max;d+=1)
            {
                SyncTable sync_table = createSyncTable(start_efs,m_frame+d,e2m); // TODO:何かの初期化関数
                PointPairs warped_point_pairs = getWarpedPointPairs(zoom, filter, filter_strength, particial_point_pairs, start_efs, particial_point_pairs.size(), sync_table);
                double warped_aaaa = getAverageAbsoluteAngularAcceleration(warped_point_pairs,estimated_angular_velocity->getFrequency());

                ld["Measured Frame"].push_back(m_frame+d);
                ld["Ratio"].push_back(warped_aaaa/raw_aaaa);

            }

            DataCollection collection("aaaa_ratio_tail.csv");
            collection.set(ld);
        }
    }
    
    return table;
}

std::vector<std::pair<int32_t, double>> VirtualGimbalManager::getSyncTableOfShortVideo()
{
    int32_t width = video_param->video_frames;
    int32_t radius = width / 2; // radius and width means number of frame in estimated angular velocity, not measured angular velocity.

    Eigen::VectorXd correlation = getCorrelationCoefficient(0, width);
    double measured_frame = getSubframeOffsetInSecond(correlation, 0, width) * measured_angular_velocity->getFrequency() + (double)radius / estimated_angular_velocity->getFrequency() * measured_angular_velocity->getFrequency();

    std::vector<std::pair<int32_t, double>> table;
    table.emplace_back(0, measured_frame + (0 - radius) / estimated_angular_velocity->getFrequency() * measured_angular_velocity->getFrequency());
    table.emplace_back(width - 1, measured_frame + ((width - 1) - radius) / estimated_angular_velocity->getFrequency() * measured_angular_velocity->getFrequency());

    return table;
}

// std::shared_ptr<ResamplerParameter> VirtualGimbalManager::getResamplerParameterWithClockError(Eigen::VectorXd &correlation_begin, Eigen::VectorXd &correlation_end)
// {
//     // TODO: Consider video length is less than 1000
//     correlation_begin = getCorrelationCoefficient(0, 1000);
//     double offset_begin = getSubframeOffsetInSecond(correlation_begin, 0, 1000);
//     correlation_end = getCorrelationCoefficient(estimated_angular_velocity->getFrames() - 1000, 1000);
//     double offset_end = getSubframeOffsetInSecond(correlation_end, estimated_angular_velocity->getFrames() - 1000, 1000);
//     double ratio = (offset_end - offset_begin) / ((estimated_angular_velocity->getFrames() - 1000) * estimated_angular_velocity->getInterval());
//     printf("offset begin: %f, offset end: %f, ratio:%f\r\n", offset_begin, offset_end, ratio);
//     double L = estimated_angular_velocity->getLengthInSecond();
//     double W = 1000.0 * estimated_angular_velocity->getInterval();
//     double &t2 = offset_end;
//     double &t1 = offset_begin;
//     double a = (t2 - t1) / (L - W);
//     printf("L:%f W:%f t1:%f t2:%f a:%f\r\n", L, W, t1, t2, a);
//     double modified_frequency = estimated_angular_velocity->getFrequency() * a;

//     Eigen::VectorXd correlation_modified = getCorrelationCoefficient(0, 1000, modified_frequency);
//     double modified_offset = getSubframeOffsetInSecond(correlation_modified, 0, 1000, modified_frequency);

//     // double modified_offset = t1/a + W*(a-1)*0.5;
//     printf("modified_frequency:%f modified_offset:%f\r\n", modified_frequency, modified_offset);
//     return std::make_shared<ResamplerParameter>(modified_frequency, modified_offset, estimated_angular_velocity->getLengthInSecond());
// }