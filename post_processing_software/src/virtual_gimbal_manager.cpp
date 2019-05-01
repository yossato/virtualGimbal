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
#include "virtual_gimbal_manager.h"

VirtualGimbalManager::VirtualGimbalManager()
{
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
    measured_angular_velocity.reset(new AngularVelocity(readSamplingRateFromJson(file_name)));
    measured_angular_velocity->data = readAngularVelocityFromJson(file_name);
    if (info)
    {
        rotateAngularVelocity(measured_angular_velocity->data, info->sd_card_rotation_);
    }
}

/**
 * @brief For angular velocity from optical flow 
 **/
void VirtualGimbalManager::setEstimatedAngularVelocity(const char *file_name, CameraInformationPtr info, int32_t maximum_synchronize_frames)
{
    std::shared_ptr<cv::VideoCapture> capture = std::make_shared<cv::VideoCapture>(file_name); //動画をオープン

    Eigen::MatrixXd optical_flow;

    calcShiftFromVideo(capture, maximum_synchronize_frames, optical_flow);

    estimated_angular_velocity.reset(new AngularVelocity(capture->get(cv::CAP_PROP_FPS)));
    estimated_angular_velocity->data.resize(optical_flow.rows(), optical_flow.cols());

    estimated_angular_velocity->data.col(0) =
        optical_flow.col(1).unaryExpr([&](double a) { return estimated_angular_velocity->getFrequency() * atan(a / (-info->fy_)); });
    estimated_angular_velocity->data.col(1) =
        optical_flow.col(0).unaryExpr([&](double a) { return estimated_angular_velocity->getFrequency() * -atan(a / (info->fx_)); });
    estimated_angular_velocity->data.col(2) = -estimated_angular_velocity->getFrequency() * optical_flow.col(2);
}

/**
 * @brief For angular velocity from chess board 
 **/
void VirtualGimbalManager::setEstimatedAngularVelocity(Eigen::MatrixXd &angular_velocity, Eigen::VectorXd &confidence, double frequency)
{
    estimated_angular_velocity = std::make_shared<AngularVelocity>(frequency);
    estimated_angular_velocity->confidence = confidence;
    estimated_angular_velocity->data = angular_velocity;
}

void VirtualGimbalManager::setRotation(const char *file_name, CameraInformation &cameraInfo)
{
    rotation.reset(new Rotation());
    // std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> angular_velocity;
    // if(file_name){
    //     readAngularVelocityFromJson(angular_velocity,file_name);
    // }else{
    //     throw  "Json file not found.";
    // }

    // angularVelocityCoordinateTransformer(angular_velocity,cameraInfo.sd_card_rotation_);
}

Eigen::MatrixXd VirtualGimbalManager::estimate()
{
    Eigen::MatrixXd measured_angular_velocity_resampled = measured_angular_velocity->getResampledData(ResamplerParameterPtr(new ResamplerParameter(video_param->getFrequency(), 0, 0)));

    std::cout << "measured_angular_velocity->data" << std::endl
              << measured_angular_velocity->data.block(measured_angular_velocity->data.rows() - 100, 0, 100, 3) << std::endl;
    std::cout << "measured_angular_velocity_resampled" << std::endl
              << measured_angular_velocity_resampled.block(measured_angular_velocity_resampled.rows() - 100, 0, 100, 3) << std::endl;
    int32_t diff = measured_angular_velocity_resampled.rows() - estimated_angular_velocity->data.rows();
    std::cout << diff << std::endl;
    std::vector<double> correlation_coefficients(diff + 1);
    for (int32_t frame = 0, end = correlation_coefficients.size(); frame < end; ++frame)
    {
        int32_t number_of_data = estimated_angular_velocity->confidence.cast<int>().array().sum();
        if (0 == number_of_data)
        {
            correlation_coefficients[frame] = std::numeric_limits<double>::max();
        }
        else
        {
            correlation_coefficients[frame] = ((measured_angular_velocity_resampled.block(frame, 0, estimated_angular_velocity->data.rows(), estimated_angular_velocity->data.cols()) - estimated_angular_velocity->data).array().colwise() * estimated_angular_velocity->confidence.array()).abs().sum() / (double)number_of_data;
        }

        if (frame % 100 == 0)
        {
            printf("\r%d / %d", frame, diff);
            std::cout << std::flush;
        }
    }
    // std::cout << correlation_coefficients.block(0,0,100,1) << std::endl;
    int32_t minimum_correlation_frame = std::distance(correlation_coefficients.begin(), min_element(correlation_coefficients.begin(), correlation_coefficients.end()));
    //最小値サブピクセル推定
    double minimum_correlation_subframe = 0.0;
    if (minimum_correlation_frame == 0)
    { //位置が最初のフレームで一致している場合
        minimum_correlation_subframe = 0.0;
    }
    else if (minimum_correlation_frame == (diff - 1))
    { //末尾
        minimum_correlation_subframe = (double)(diff - 1);
    }
    else
    { //その他
        minimum_correlation_subframe = -(correlation_coefficients[minimum_correlation_frame + 1] - correlation_coefficients[minimum_correlation_frame - 1]) / (2 * correlation_coefficients[minimum_correlation_frame - 1] - 4 * correlation_coefficients[minimum_correlation_frame] + 2 * correlation_coefficients[minimum_correlation_frame + 1]);
    }
    minimum_correlation_subframe += (double)minimum_correlation_frame;
    std::cout << std::endl
              << minimum_correlation_subframe << std::endl;

    resampler_parameter_ = std::make_shared<ResamplerParameter>(video_param->getFrequency(), minimum_correlation_subframe / video_param->getFrequency(), estimated_angular_velocity->data.rows() / estimated_angular_velocity->getFrequency());

    Eigen::MatrixXd retval(correlation_coefficients.size(), 1);
    for (int i = 0, e = correlation_coefficients.size(); i < e; ++i)
    {
        retval(i, 0) = correlation_coefficients[i];
    }
    return retval;
}

Eigen::MatrixXd VirtualGimbalManager::getSynchronizedMeasuredAngularVelocity()
{
    Eigen::MatrixXd data;
    data.resize(estimated_angular_velocity->data.rows(), estimated_angular_velocity->data.cols() + measured_angular_velocity->data.cols());
    data.block(0, 0, estimated_angular_velocity->data.rows(), estimated_angular_velocity->data.cols()) = estimated_angular_velocity->data;
    Eigen::MatrixXd resampled = measured_angular_velocity->getResampledData(resampler_parameter_);
    assert(estimated_angular_velocity->data.rows() == resampled.rows());
    data.block(0, estimated_angular_velocity->data.cols(), resampled.rows(), resampled.cols()) = resampled;
    return data;
}

Eigen::MatrixXd VirtualGimbalManager::getRotationQuaternions()
{
    Eigen::MatrixXd data;
    data.resize(estimated_angular_velocity->data.rows(), 4);
    rotation_quaternion = std::make_shared<RotationQuaternion>(measured_angular_velocity, *resampler_parameter_);
    for (int i = 0, e = data.rows(); i < e; ++i)
    {

        Eigen::MatrixXd temp = rotation_quaternion->getRotationQuaternion((double)i * video_param->getInterval()).coeffs().transpose();
        // std::cout << rotation_quaternion->getRotationQuaternion((double)i * video_param->getInterval()).coeffs().transpose() << std::endl;
        // std::cout << temp << std::endl << std::flush;
        data.row(i) = temp;
    }
    return data;
}

// void VirtualGimbalManager::getEstimatedAndMeasuredAngularVelocity(Eigen::MatrixXd &data){
//     data.resize(estimated_angular_velocity->data.rows(),estimated_angular_velocity->data.cols()+measured_angular_velocity->data.cols());
//     data.block(0,0,estimated_angular_velocity->data.rows(),estimated_angular_velocity->data.cols()) = estimated_angular_velocity->data;
//     resampler_parameter rp =
//     // data.block(0,estimated_angular_velocity->data.cols(),measured_angular_velocity) =
// }

std::map<int, std::vector<cv::Point2f>> VirtualGimbalManager::getCornerDictionary(cv::Size &pattern_size, bool debug_speedup, bool Verbose)
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
    return corner_dict;
}

Eigen::MatrixXd VirtualGimbalManager::estimateAngularVelocity(const std::map<int, std::vector<cv::Point2f>> &corner_dict, const std::vector<cv::Point3f> &world_points, Eigen::VectorXd &confidence)
{
    cv::Mat CameraMatrix = (cv::Mat_<float>(3, 3) << video_param->camera_info->fx_, 0, video_param->camera_info->cx_, 0, video_param->camera_info->fy_, video_param->camera_info->cy_, 0, 0, 1);
    cv::Mat DistCoeffs = (cv::Mat_<float>(1, 4) << video_param->camera_info->k1_, video_param->camera_info->k2_, video_param->camera_info->p1_, video_param->camera_info->p2_);
    std::map<int, cv::Mat> RotationVector;
    std::map<int, cv::Mat> TranslationVector;

    confidence = Eigen::VectorXd::Zero(video_param->video_frames);

    Eigen::MatrixXd estimated_angular_velocity = Eigen::MatrixXd::Zero(video_param->video_frames, 3);
    for (const auto &el : corner_dict)
    {
        cv::solvePnP(world_points, el.second, CameraMatrix, DistCoeffs, RotationVector[el.first], TranslationVector[el.first]);
        // printf("%d,%f,%f,%f ",el.first,RotationVector[el.first].at<float>(0,0),RotationVector[el.first].at<float>(1,0),RotationVector[el.first].at<float>(2,0));
        // std::cout << "tvec:\r\n" << TranslationVector[el.first] << std::endl << std::flush;
        // std::cout << "rvec:\r\n" << RotationVector[el.first] << std::endl << std::flush;

        Eigen::Quaterniond rotation_quaternion = Vector2Quaternion<double>(
                                                     Eigen::Vector3d(RotationVector[el.first].at<float>(0, 0), RotationVector[el.first].at<float>(1, 0), RotationVector[el.first].at<float>(2, 0)))
                                                     .conjugate();
        printf("%d,%f,%f,%f,%f,", el.first, rotation_quaternion.x(), rotation_quaternion.y(), rotation_quaternion.z(), rotation_quaternion.w());
        if (0 != RotationVector.count(el.first - 1))
        {
            Eigen::Quaterniond rotation_quaternion_previous = Vector2Quaternion<double>(
                                                                  Eigen::Vector3d(RotationVector[el.first - 1].at<float>(0, 0), RotationVector[el.first - 1].at<float>(1, 0), RotationVector[el.first - 1].at<float>(2, 0)))
                                                                  .conjugate();
            // cv::Mat diff = RotationVector[el.first]-RotationVector[el.first-1];
            Eigen::Quaterniond diff = rotation_quaternion * rotation_quaternion_previous.conjugate();
            // printf("%f,%f,%f\n",diff.at<float>(0,0),diff.at<float>(1,0),diff.at<float>(2,0));
            printf("%f,%f,%f,%f\n", diff.x(), diff.y(), diff.z(), diff.w());
            Eigen::Vector3d diff_vector = Quaternion2Vector(diff);
            Eigen::Quaterniond estimated_angular_velocity_in_board_coordinate(0.0, diff_vector[0], diff_vector[1], diff_vector[2]);
            Eigen::Quaterniond estimated_angular_velocity_in_camera_coordinate = (rotation_quaternion.conjugate() * estimated_angular_velocity_in_board_coordinate * rotation_quaternion);
            estimated_angular_velocity.row(el.first) << estimated_angular_velocity_in_camera_coordinate.x(), estimated_angular_velocity_in_camera_coordinate.y(),
                estimated_angular_velocity_in_camera_coordinate.z();
            confidence(el.first) = 1.0;
        }
        else
        {
            printf("0,0,0\n");
        }
    }
    std::cout << std::flush;

    return estimated_angular_velocity * video_param->getFrequency();
}

/**
 * @brief Undistort and unrolling chess board board points. 
 **/
void VirtualGimbalManager::getUndistortUnrollingChessBoardPoints(double time, const std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst)
{
    //手順
    //1.補正前画像を分割した時の分割点の座標(pixel)を計算
    //2.1の座標を入力として、各行毎のW(t1,t2)を計算
    //3.補正後の画像上のポリゴン座標(pixel)を計算、歪み補正も含める

    for (const auto &el : src) //(int j = 0; j <= division_y; ++j)
    {
        //W(t1,t2)を計算
        //1
        double v = el.y; //(double)j / division_y * camera_info_.height_;

        double time_in_row = video_param->camera_info->rolling_shutter_coefficient_ * (v - video_param->camera_info->height_*0.5) / video_param->camera_info->height_;
        Eigen::MatrixXd R = (rotation_quaternion->getRotationQuaternion(time_in_row + time).conjugate() * rotation_quaternion->getRotationQuaternion(time)) .matrix();
        {
            double u = el.x; //(double)i / division_x * camera_info_.width_;
            //後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
            Eigen::Vector3d p;
            p << (u - video_param->camera_info->cx_) / video_param->camera_info->fx_, (v - video_param->camera_info->cy_) / video_param->camera_info->fy_, 1.0; // Homogenious coordinate
            //2
            Eigen::MatrixXd XYW = R * p;

            double x1 = XYW(0, 0) / XYW(2, 0);
            double y1 = XYW(1, 0) / XYW(2, 0);

            double r = sqrt(x1 * x1 + y1 * y1);

            double x2 = x1 * (1.0 + video_param->camera_info->inverse_k1_ * r * r + video_param->camera_info->inverse_k2_ * r * r * r * r) + 2.0 * video_param->camera_info->inverse_p1_ * x1 * y1 + video_param->camera_info->inverse_p2_ * (r * r + 2.0 * x1 * x1);
            double y2 = y1 * (1.0 + video_param->camera_info->inverse_k1_ * r * r + video_param->camera_info->inverse_k2_ * r * r * r * r) + video_param->camera_info->inverse_p1_ * (r * r + 2.0 * y1 * y1) + 2.0 * video_param->camera_info->inverse_p2_ * x1 * y1;
            //変な折り返しを防止
            if ((pow(x2 - x1, 2) > 1.0) || (pow(y2 - y1, 2) > 1.0))
            {
                //                printf("折り返し防止\r\n");
                x2 = x1;
                y2 = y1;
            }
            dst.push_back(cv::Point2f(
                x2 * video_param->camera_info->fx_ + video_param->camera_info->cx_,
                y2 * video_param->camera_info->fy_ + video_param->camera_info->cy_));
        }
    }
    return;
}