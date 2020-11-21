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
    measured_angular_velocity.reset(new AngularVelocity(readSamplingRateFromJson(file_name)));
    measured_angular_velocity->data = readAngularVelocityFromJson(file_name);
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

Eigen::VectorXd VirtualGimbalManager::getCorrelationCoefficient(int32_t begin, int32_t length, double frequency)
{
    if (0 == length)
    {
        length = estimated_angular_velocity->data.rows();
    }

    if (frequency <= std::numeric_limits<double>::epsilon())
    {
        frequency = video_param->getFrequency();
    }
    Eigen::MatrixXd measured_angular_velocity_resampled = measured_angular_velocity->getResampledData(ResamplerParameterPtr(new ResamplerParameter(frequency, 0, 0)));
    assert(length <= estimated_angular_velocity->data.rows());

    Eigen::MatrixXd particial_estimated_angular_velocity = estimated_angular_velocity->data.block(begin, 0, length, estimated_angular_velocity->data.cols());
    Eigen::VectorXd particial_confidence = estimated_angular_velocity->confidence.block(begin, 0, length, estimated_angular_velocity->confidence.cols());

    int32_t diff = measured_angular_velocity_resampled.rows() - particial_estimated_angular_velocity.rows();
    if(0 >= diff){
        std::cerr << "Error: Measured angular velocity data from a gyroscope sensor is shorter than video length.\r\nPlease confirm input json file of angular velocity\r\n" 
        << "Length of angular velocity is " <<  measured_angular_velocity->getLengthInSecond() << " seconds.\r\n"
        << "Length of video is " << estimated_angular_velocity->getLengthInSecond() << " seconds.\r\n" << std::endl << std::flush;
        throw;
    }
    Eigen::VectorXd correlation_coefficients(diff + 1);
    for (int32_t frame = 0, end = correlation_coefficients.rows(); frame < end; ++frame)
    {
        int32_t number_of_data = particial_confidence.cast<int>().array().sum();
        if (0 == number_of_data)
        {
            correlation_coefficients[frame] = std::numeric_limits<double>::max();
        }
        else
        {
            correlation_coefficients[frame] = ((measured_angular_velocity_resampled.block(frame, 0, particial_estimated_angular_velocity.rows(), particial_estimated_angular_velocity.cols()) - particial_estimated_angular_velocity).array().colwise() * particial_confidence.array()).abs().sum() / (double)number_of_data;
        }

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
    if (0 == length)
    {
        length = estimated_angular_velocity->data.rows();
    }
    if (frequency < std::numeric_limits<double>::epsilon())
    {
        frequency = video_param->getFrequency();
    }
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

void VirtualGimbalManager::setResamplerParameter(double start, double new_frequency)
{
    if (0.0 == new_frequency)
    {
        resampler_parameter_ = std::make_shared<ResamplerParameter>(video_param->getFrequency(), start, estimated_angular_velocity->data.rows() * estimated_angular_velocity->getInterval());
    }
    else
    {
        resampler_parameter_ = std::make_shared<ResamplerParameter>(new_frequency, start, estimated_angular_velocity->data.rows() * estimated_angular_velocity->getInterval());
    }
}

void VirtualGimbalManager::setResamplerParameter(ResamplerParameterPtr param)
{
    resampler_parameter_ = param;
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

        data.row(i) = temp;
    }
    return data;
}

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

Eigen::MatrixXd VirtualGimbalManager::estimateAngularVelocity(const std::map<int, std::vector<cv::Point2d>> &corner_dict, const std::vector<cv::Point3d> &world_points, Eigen::VectorXd &confidence)
{
    cv::Mat CameraMatrix = (cv::Mat_<double>(3, 3) << video_param->camera_info->fx_, 0, video_param->camera_info->cx_, 0, video_param->camera_info->fy_, video_param->camera_info->cy_, 0, 0, 1);
    cv::Mat DistCoeffs = (cv::Mat_<double>(1, 4) << video_param->camera_info->k1_, video_param->camera_info->k2_, video_param->camera_info->p1_, video_param->camera_info->p2_);
    std::map<int, cv::Mat> RotationVector;
    std::map<int, cv::Mat> TranslationVector;

    confidence = Eigen::VectorXd::Zero(video_param->video_frames);

    Eigen::MatrixXd estimated_angular_velocity = Eigen::MatrixXd::Zero(video_param->video_frames, 3);
    for (const auto &el : corner_dict)
    {
        cv::solvePnP(world_points, el.second, CameraMatrix, DistCoeffs, RotationVector[el.first], TranslationVector[el.first]);

        Eigen::Quaterniond rotation_quaternion = Vector2Quaternion<double>(
                                                     Eigen::Vector3d(RotationVector[el.first].at<double>(0, 0), RotationVector[el.first].at<double>(1, 0), RotationVector[el.first].at<double>(2, 0)))
                                                     .conjugate();

        if (0 != RotationVector.count(el.first - 1))
        {
            Eigen::Quaterniond rotation_quaternion_previous = Vector2Quaternion<double>(
                                                                  Eigen::Vector3d(RotationVector[el.first - 1].at<double>(0, 0), RotationVector[el.first - 1].at<double>(1, 0), RotationVector[el.first - 1].at<double>(2, 0)))
                                                                  .conjugate();
            Eigen::Quaterniond diff = rotation_quaternion * rotation_quaternion_previous.conjugate();

            Eigen::Vector3d diff_vector = Quaternion2Vector(diff);
            Eigen::Quaterniond estimated_angular_velocity_in_board_coordinate(0.0, diff_vector[0], diff_vector[1], diff_vector[2]);
            Eigen::Quaterniond estimated_angular_velocity_in_camera_coordinate = (rotation_quaternion.conjugate() * estimated_angular_velocity_in_board_coordinate * rotation_quaternion);
            estimated_angular_velocity.row(el.first) << estimated_angular_velocity_in_camera_coordinate.x(), estimated_angular_velocity_in_camera_coordinate.y(),
                estimated_angular_velocity_in_camera_coordinate.z();
            confidence(el.first) = 1.0;
        }
    }

    return estimated_angular_velocity * video_param->getFrequency();
}

/**
 * @brief Estimate angular velocity from video optical flow
 **/
void VirtualGimbalManager::estimateAngularVelocity(Eigen::MatrixXd &estimated_angular_velocity, Eigen::MatrixXd &confidence)
{
    Eigen::MatrixXd optical_flow;
    if (jsonExists(video_param->video_file_name))
    {
        readOpticalFlowFromJson(video_param->video_file_name, optical_flow, confidence);
    }
    else
    {
        CalcShiftFromVideo(video_param->video_file_name.c_str(), video_param->video_frames, optical_flow, confidence);
    }
    estimated_angular_velocity.resize(optical_flow.rows(), optical_flow.cols());
    estimated_angular_velocity.col(0) =
        optical_flow.col(1).unaryExpr([&](double a) { return video_param->getFrequency() * atan(a / (video_param->camera_info->fy_)); });
    estimated_angular_velocity.col(1) =
        optical_flow.col(0).unaryExpr([&](double a) { return video_param->getFrequency() * -atan(a / (video_param->camera_info->fx_)); });
    estimated_angular_velocity.col(2) = -video_param->getFrequency() * optical_flow.col(2);
}

void VirtualGimbalManager::getUndistortUnrollingChessBoardPoints(double time_offset, const std::pair<int, std::vector<cv::Point2d>> &corner_dict, std::vector<cv::Point2d> &dst, double line_delay)
{
    getUndistortUnrollingChessBoardPoints(corner_dict.first * video_param->getInterval() + time_offset, corner_dict.second, dst, line_delay);
}

/**
 * @brief Undistort and unrolling chess board board points. 
 **/
void VirtualGimbalManager::getUndistortUnrollingChessBoardPoints(double time, const std::vector<cv::Point2d> &src, std::vector<cv::Point2d> &dst, double line_delay)
{

    // Collect time difference between video frame and gyro frame. These frame rates are deferent, so that time should be compensated.
    time += (measured_angular_velocity->getInterval() - estimated_angular_velocity->getInterval()) * 0.5;
    //手順
    //1.補正前画像を分割した時の分割点の座標(pixel)を計算
    //2.1の座標を入力として、各行毎のW(t1,t2)を計算
    //3.補正後の画像上のポリゴン座標(pixel)を計算、歪み補正も含める

    for (const auto &el : src) //(int j = 0; j <= division_y; ++j)
    {
        //W(t1,t2)を計算
        //1
        double v = el.y;

        double time_in_row = line_delay * (v - video_param->camera_info->height_ * 0.5);
        Eigen::MatrixXd R = (rotation_quaternion->getRotationQuaternion(time_in_row + time).conjugate() * rotation_quaternion->getRotationQuaternion(time)).matrix();
        {
            double u = el.x;
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
            dst.push_back(cv::Point2d(
                x2 * video_param->camera_info->fx_ + video_param->camera_info->cx_,
                y2 * video_param->camera_info->fy_ + video_param->camera_info->cy_));
        }
    }
    return;
}

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

void VirtualGimbalManager::spin(double zoom, FilterPtr filter, Eigen::VectorXd &filter_strength, std::vector<std::pair<int32_t, double>> &sync_table, bool show_image)
{

    // Prepare correction rotation matrix generator. This constructor run a thread.
    MultiThreadRotationMatrixGenerator gen(video_param, filter, measured_angular_velocity, filter_strength, sync_table,queue_size_);

    // Prepare OpenCL
    cv::ocl::Context context;
    cv::Mat mat_src = cv::Mat::zeros(video_param->camera_info->height_, video_param->camera_info->width_, CV_8UC4); // TODO:冗長なので書き換える
    cv::UMat umat_src = mat_src.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    cv::String build_opt;
    initializeCL(context);

    // Open Video
    reader_ = std::make_shared<MultiThreadVideoReader>(video_param->video_file_name,queue_size_);
    auto capture = getVideoCapture();

    // Stabilize every frames
    float ik1 = video_param->camera_info->inverse_k1_;
    float ik2 = video_param->camera_info->inverse_k2_;
    float ip1 = video_param->camera_info->inverse_p1_;
    float ip2 = video_param->camera_info->inverse_p2_;
    float fx = video_param->camera_info->fx_;
    float fy = video_param->camera_info->fy_;
    float cx = video_param->camera_info->cx_;
    float cy = video_param->camera_info->cy_;

    for (int frame = 0; frame <= video_param->video_frames; ++frame)
    {
        UMatPtr umat_src;
        reader_->get(umat_src);
        if (!umat_src)
        {
            break;
        }

        LAP_BEGIN
        MatrixPtr R;
        gen.get(R);
        LAP

            cv::ocl::Image2D image(*umat_src);
        LAP
            UMatPtr umat_dst_ptr(new cv::UMat(mat_src.size(), CV_8UC4, cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY));
        LAP
            cv::Mat mat_R = cv::Mat(R->size(), 1, CV_32F, R->data());
        LAP
            cv::UMat umat_R = mat_R.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
        LAP
            cv::ocl::Kernel kernel;
        getKernel(kernel_name, kernel_function, kernel, context, build_opt);
        LAP
            kernel.args(image, cv::ocl::KernelArg::WriteOnly(*umat_dst_ptr), cv::ocl::KernelArg::ReadOnlyNoSize(umat_R),
                        (float)zoom,
                        ik1,
                        ik2,
                        ip1,
                        ip2,
                        fx,
                        fy,
                        cx,
                        cy);
        size_t globalThreads[3] = {(size_t)mat_src.cols, (size_t)mat_src.rows, 1};
        //size_t localThreads[3] = { 16, 16, 1 };
        LAP bool success = kernel.run(3, globalThreads, NULL, true);
        if (!success)
        {
            cout << "Failed running the kernel..." << endl
                 << flush;
            throw "Failed running the kernel...";
        }

        LAP
        // Show image on display
        if (show_image)
        {
            cv::UMat small, small_src;
            cv::resize(*umat_dst_ptr, small, cv::Size(), 0.5, 0.5);
            cv::resize(*umat_src, small_src, cv::Size(), 0.5, 0.5);
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
                    return;
                }
            }
        }

        if (writer_)
        {
            writer_->push(umat_dst_ptr);
        }
        LAP
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
        LAP
    }
    cv::destroyAllWindows();
    return;
}

#undef LAP_BEGIN
#undef LAP

int VirtualGimbalManager::spinInpainting(double zoom, std::vector<std::pair<int32_t, double>> &sync_table, FilterPtr filter, int filter_strength, bool show_image)
{
    // Prepare OpenCL
    cv::ocl::Context context;
    initializeCL(context);


    // Prepare
    measured_angular_velocity->calculateAngleQuaternion();

    // UMatの準備
    // UMatのバッファ
    UMatMap b;

    size_t buffer_size = 21;
    for(size_t i=0;i<buffer_size/2 ;++i)
    {
        reader_->get(b[i]);
    }

    // BGRAのバッファ2枚 (b_past,b_future)
    UMatPtr b_past(new cv::UMat(b[0]->size(), CV_8UC4, cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY));
    UMatPtr b_future(new cv::UMat(b[0]->size(), CV_8UC4, cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY));

    // 出力画像のBGRAの1枚 (b_output)
    UMatPtr b_output(new cv::UMat(b[0]->size(), CV_8UC4, cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY));


    constexpr bool OPENCL_SUCCESS = true;

    if(show_image)
    {
        cv::namedWindow("Original",cv::WINDOW_NORMAL);
        cv::namedWindow("Result",cv::WINDOW_NORMAL);
    }

    // TODO: MultiThreadVideoReaderがすでに内部にbufferをもっているので、こいつを借りてもいい気がする
    for(int frame = 0; frame < video_param->video_frames ; ++frame)
    {
        // 動画最も古い1フレームを削除
        while(b.size()>buffer_size)
        {
            b.erase(b.begin());
        }
        // 動画最新1フレーム読み込み
        int back_index = frame+buffer_size/2;
        UMatPtr p;
        reader_->get(p);
        if(p)
        {
            b[back_index] = std::move(p);
        }

        // フィルタ済み姿勢計算 @ 注目フレームの基準時間
        // getRelativeAngleで前後のフレームは得られるらしい
        // Privateなので直接アクセスできないorz
        // 以下の関数で、補正用のquaternionと、相対角度のquaternionを、estimatedな時間で取得する
        // std::vector<Eigen::Quaterniond> measured_angle_quaternions(video_param->camera_info->height_*buffer_size);
        Eigen::Quaterniond stabilized_angle_quaternion;

        measured_angular_velocity->getCorrectionQuaternion(frame, filter->getFilterCoefficient(filter_strength)
        , sync_table, stabilized_angle_quaternion);
        
        // UMatとしてつくる　col は 時間軸(フレーム、行数)　rowはバッファーのサイズ(=フレーム数)に対応
        // static cv::UMat correction_matrices(cv::Size(video_param->camera_info->height_ * 9, buffer_size),CV_32F, cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY);

        

        // b_futureについて基準フレームから未来方向へfor文で連続して画素を埋めていく
        b_future->setTo(cv::Scalar(127,127,127,255)); // BGR A, A channel means distance from target frame
        for(int future_frame = frame + buffer_size/2; frame <= future_frame; --future_frame)
        {
            if(b.count(future_frame) == 0)
            {
                continue;
            }
            std::vector<float> stabilized_angle_matrices;
            measured_angular_velocity->getCorrectionMatrices(stabilized_angle_quaternion, future_frame, video_param->camera_info->height_, video_param->camera_info->line_delay_ * video_param->getFrequency(), sync_table, stabilized_angle_matrices);
            assert(future_frame >= frame);
            int distance = future_frame - frame;
            if(OPENCL_SUCCESS !=  fillPixelValues(context, zoom, stabilized_angle_matrices,distance,b[future_frame],b_future))
            {
                std::cerr << "Failed to run a fill_function kernel at Line:" << __LINE__ << " of " << __FILE__ << std::endl;
                return -1;
            }
        }

        // b_pastについて基準フレームから過去方向へfor文で連続して画素を埋めていく
        b_past->setTo(cv::Scalar(127,127,127,255));
        for(int past_frame = frame - buffer_size/2; past_frame <= frame; ++past_frame)
        {
            if(b.count(past_frame) == 0)
            {
                continue;
            }
            std::vector<float> stabilized_angle_matrices;
            measured_angular_velocity->getCorrectionMatrices(stabilized_angle_quaternion, past_frame, video_param->camera_info->height_, video_param->camera_info->line_delay_ * video_param->getFrequency(), sync_table, stabilized_angle_matrices);
            assert(frame >= past_frame);
            int distance = frame - past_frame;
            if(OPENCL_SUCCESS != fillPixelValues(context, zoom,stabilized_angle_matrices,distance,b[past_frame],b_past))
            {
                std::cerr << "Failed to run a fill_function kernel at Line:" << __LINE__ << " of " << __FILE__ << std::endl;
                return -1;
            }
        }

        // 最後に b_pastとb_futureからb_outputを生成
        if(OPENCL_SUCCESS != interpolatePixels(context, b_past,b_future,b_output))
        {
            std::cerr << "Failed to run a interpoloate_function kernel at Line:" << __LINE__ << " of " << __FILE__ << std::endl;
            return -1;
        }

        // Show image on displayreturn 0;
        if (show_image)
        {
            cv::imshow("Original", *p);
            cv::imshow("Result", *b_output);
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
    cv::Mat mat_params = (cv::Mat_<float>(9,1) << (float)zoom, ik1, ik2, ip1, ip2, fx, fy, cx, cy);
    cv::UMat umat_params = mat_params.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);

    cv::String build_opt;
    // initializeCL(context);

    cv::ocl::Image2D image(*source_image);

    cv::ocl::Kernel kernel;
    getKernel(kernel_name, "fill_function", kernel, context, build_opt);
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
    getKernel(kernel_name, "interpoloate_function", kernel, context, build_opt);
    kernel.args(past_image,
                future_image,
                cv::ocl::KernelArg::WriteOnly(*output)
                );
    size_t globalThreads[3] = {(size_t)output->cols, (size_t)output->rows, 1};
    bool success = kernel.run(3, globalThreads, NULL, true);
    return success;
}


void VirtualGimbalManager::enableWriter(const char *video_path)
{
    writer_ = std::make_shared<MultiThreadVideoWriter>(MultiThreadVideoWriter::getOutputName(video_path), *video_param,queue_size_);
}

std::vector<std::pair<int32_t, double>> VirtualGimbalManager::getSyncTable(double period_in_second, int32_t width)
{
    assert(width % 2);          // Odd
    int32_t radius = width / 2; // radius and width means number of frame in estimated angular velocity, not measured angular velocity.
    std::vector<std::pair<int32_t, double>> table;
    for (int center = radius, e = estimated_angular_velocity->getFrames() - radius; center < e; center += (int32_t)(period_in_second*video_param->getFrequency()))
    {
        Eigen::VectorXd correlation = getCorrelationCoefficient(center - radius, width);
        double measured_frame = getSubframeOffsetInSecond(correlation, center - radius, width) * measured_angular_velocity->getFrequency() + (double)radius / estimated_angular_velocity->getFrequency() * measured_angular_velocity->getFrequency();
        table.emplace_back(center, measured_frame);
    }
    return table;
}

std::vector<std::pair<int32_t, double>> VirtualGimbalManager::getSyncTableOfShortVideo(){
    int32_t width = video_param->video_frames;
    int32_t radius = width / 2; // radius and width means number of frame in estimated angular velocity, not measured angular velocity.

    Eigen::VectorXd correlation = getCorrelationCoefficient(0, width);
    double measured_frame = getSubframeOffsetInSecond(correlation, 0, width) * measured_angular_velocity->getFrequency() + (double)radius / estimated_angular_velocity->getFrequency() * measured_angular_velocity->getFrequency();

    std::vector<std::pair<int32_t, double>> table;
    table.emplace_back(0,measured_frame+(0-radius) / estimated_angular_velocity->getFrequency() * measured_angular_velocity->getFrequency());
    table.emplace_back(width-1,measured_frame+((width-1)-radius) / estimated_angular_velocity->getFrequency() * measured_angular_velocity->getFrequency());

    return table;
}

std::shared_ptr<ResamplerParameter> VirtualGimbalManager::getResamplerParameterWithClockError(Eigen::VectorXd &correlation_begin, Eigen::VectorXd &correlation_end)
{
    // TODO: Consider video length is less than 1000
    correlation_begin = getCorrelationCoefficient(0, 1000);
    double offset_begin = getSubframeOffsetInSecond(correlation_begin, 0, 1000);
    correlation_end = getCorrelationCoefficient(estimated_angular_velocity->getFrames() - 1000, 1000);
    double offset_end = getSubframeOffsetInSecond(correlation_end, estimated_angular_velocity->getFrames() - 1000, 1000);
    double ratio = (offset_end - offset_begin) / ((estimated_angular_velocity->getFrames() - 1000) * estimated_angular_velocity->getInterval());
    printf("offset begin: %f, offset end: %f, ratio:%f\r\n", offset_begin, offset_end, ratio);
    double L = estimated_angular_velocity->getLengthInSecond();
    double W = 1000.0 * estimated_angular_velocity->getInterval();
    double &t2 = offset_end;
    double &t1 = offset_begin;
    double a = (t2 - t1) / (L - W);
    printf("L:%f W:%f t1:%f t2:%f a:%f\r\n", L, W, t1, t2, a);
    double modified_frequency = estimated_angular_velocity->getFrequency() * a;

    Eigen::VectorXd correlation_modified = getCorrelationCoefficient(0, 1000, modified_frequency);
    double modified_offset = getSubframeOffsetInSecond(correlation_modified, 0, 1000, modified_frequency);

    // double modified_offset = t1/a + W*(a-1)*0.5;
    printf("modified_frequency:%f modified_offset:%f\r\n", modified_frequency, modified_offset);
    return std::make_shared<ResamplerParameter>(modified_frequency, modified_offset, estimated_angular_velocity->getLengthInSecond());
}