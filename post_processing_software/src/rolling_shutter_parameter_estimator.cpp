#include <stdio.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <memory>
#include "mINIRead.hpp"
#include "vsp.h"
#include "visualizer.h"
#include "json_tools.hpp"
#include "rotation_param.h"
#include "virtual_gimbal_manager.h"
#include "line_delay_estimator.hpp"

std::string getVideoSize(const char *videoName)
{
    std::shared_ptr<cv::VideoCapture> Capture = std::make_shared<cv::VideoCapture>(videoName); //動画をオープン
    assert(Capture->isOpened());
    std::string videoSize = std::to_string((int)Capture->get(cv::CAP_PROP_FRAME_WIDTH)) + std::string("x") + std::to_string((int)Capture->get(cv::CAP_PROP_FRAME_HEIGHT));
    return videoSize;
}

// std::map<int, std::vector<cv::Point2d>> getCornerDictionary(std::shared_ptr<cv::VideoCapture> capture, cv::Size &pattern_size, bool debug_speedup, bool Verbose);

int main(int argc, char **argv)
{
    //引数の確認
    char *videoPass = NULL;
    char *cameraName = NULL;
    char *lensName = NULL;
    char *jsonPass = NULL;
    bool debug_speedup = false;
    int opt;
    //    Eigen::Quaterniond camera_rotation;

    while ((opt = getopt(argc, argv, "j:i:c:l:d::")) != -1)
    {
        switch (opt)
        {
        case 'j': //input json file from virtual gimbal
            jsonPass = optarg;
            break;
        case 'i': //input video file pass
            videoPass = optarg;
            break;
        case 'c':
            cameraName = optarg;
            break;
        case 'l':
            lensName = optarg;
            break;
        case 'd':
            debug_speedup = true;
            break;

        default:
            //            printf(     "virtualGimbal\r\n"
            //                        "Hyper fast video stabilizer\r\n\r\n"
            //                        "usage: virtualGimbal [-i video] [-f angularVelocity] [[output option] -o] [options]\r\n"
            //                        );
            return 1;
        }
    }

    VirtualGimbalManager manager;
    shared_ptr<CameraInformation> camera_info(new CameraInformationJsonParser(cameraName, lensName, getVideoSize(videoPass).c_str()));
    manager.setMeasuredAngularVelocity(jsonPass, camera_info);
    manager.setVideoParam(videoPass, camera_info);
    manager.setRotation(jsonPass, *camera_info); //なんか変だぞ？

    //角速度を読み込み
    //角速度を同期
    //角度を保存
    std::shared_ptr<cv::VideoCapture> capture = std::make_shared<cv::VideoCapture>(videoPass);
    if (!capture->isOpened())
    {
        printf("Video not found.\n");
        return -1;
    }

    //動画読み込み準備ここまで

    //チェッカーボードの設定iniファイルを読み込み
    char path[512];
    getcwd(path, sizeof(path));
    std::string path_string(path);
    path_string = path_string + "/chess_board_settings.ini";
    if (ReadINIs(path_string.c_str(), INICheckerBoardParamNum, INICheckerBoardValueNames, Dcbp) != 0)
    {
        //	system("pause");	//ユーザのキーボード入力を待機
        return 1;
    }

    //キャリブレーションの準備
    cv::Size PatternSize = cv::Size((int)Dcbp.NumberOfInnerCorners.X, (int)Dcbp.NumberOfInnerCorners.Y);
    std::vector<std::vector<cv::Point2d>> imagePoints; // チェッカー交点座標を格納するベクトルのベクトル インデックスの並びは[撮影画像番号][点のIndex]

    //キャリブレーションの準備ここまで
    std::map<int, std::vector<cv::Point2d>> corner_dict = manager.getCornerDictionary(PatternSize, debug_speedup, true);

    std::vector<cv::Point3d> world_points; // チェッカー交点座標と対応する世界座標の値を格納する行列
    // 世界座標を決める
    for (int j = 0; j < PatternSize.area(); j++)
    { //チェッカーボードの交点座標を記録
        world_points.push_back(cv::Point3d(static_cast<double>(j % PatternSize.width * Dcbp.SizeOfQuadsX_mm),
                                           static_cast<double>(j / PatternSize.width * Dcbp.SizeOfQuadsY_mm),
                                           0.0));
    }

    Eigen::VectorXd confidence;
    Eigen::MatrixXd estimated_angular_velocity = manager.estimateAngularVelocity(corner_dict, world_points, confidence);
    std::cout << estimated_angular_velocity << std::endl
              << std::flush;
    manager.setEstimatedAngularVelocity(estimated_angular_velocity, confidence, capture->get(cv::CAP_PROP_FPS));

    Eigen::MatrixXd correlation = manager.estimate();
    std::vector<string> legends_angular_velocity = {"c"};
    vgp::plot(correlation, "correlation", legends_angular_velocity);

    Eigen::MatrixXd mat = manager.getSynchronizedMeasuredAngularVelocity();

    vgp::plot(mat, "Compare angular velocity", legends_angular_velocity);
    vgp::plot(mat.block(0, 0, mat.rows(), 3), "Estimated", legends_angular_velocity);
    vgp::plot(mat.block(0, 3, mat.rows(), 3), "Measured", legends_angular_velocity);

    mat = manager.getRotationQuaternions();
    vgp::plot(mat, "Rotation quaternion", legends_angular_velocity);

    // Optimize
    Eigen::VectorXd undistortion_params = Eigen::VectorXd::Zero(2);
    line_delay_functor functor(undistortion_params.size(), corner_dict.size() * (corner_dict.begin()->second.size()) * 2, camera_info, world_points, corner_dict, manager);
    Eigen::NumericalDiff<line_delay_functor> numeric_diff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<line_delay_functor>> lm(numeric_diff);
    int info = 4;
    double initial_factor = 0.1;
    while (info != Eigen::ComputationInfo::Success)
    {
        printf("Factor : %f", initial_factor);
        lm.resetParameters();
        lm.parameters.factor = initial_factor; //step bound for the diagonal shift, is this related to damping parameter, lambda?
        int info = lm.minimize(undistortion_params);
        switch (info)
        {
        case Eigen::ComputationInfo::Success:
            std::cout << "Success:" << undistortion_params << std::endl;
            break;
        case Eigen::ComputationInfo::InvalidInput:
            std::cout << "Error : InvalidInput" << std::endl;
            // return -1;
            break;
        case Eigen::ComputationInfo::NoConvergence:
            std::cout << "Error : NoConvergence:" << undistortion_params << std::endl;
            return -1;
            break;
        case Eigen::ComputationInfo::NumericalIssue:
            std::cout << "Error : NumericalIssue" << std::endl;
            // return -1;
            break;
        default:
            std::cout << "Default :" << undistortion_params << std::endl;
            break;
        }
        initial_factor *= 1.1;
    }

    //2Dでパラメータを変化させながらどうなるか試してみる
    //結果をdoubleのmatrixに入れる。
    int32_t image_width = 30;
    cv::Mat optimize_result_mat = cv::Mat::zeros(image_width, image_width, CV_32FC1);
    for (int i = 0, ie = image_width; i < ie; ++i)
    {
        for (int k = 0, ke = image_width; k < ke; ++k)
        {
            // timeとrolling shutter parameterを変化させる
            double time_offset = 1.0 / capture->get(cv::CAP_PROP_FPS) / ((double)image_width * 0.5) * (double)(i - image_width * 0.5);
            double rolling_shutter_parameter = 1.0 / capture->get(cv::CAP_PROP_FPS) / ((double)image_width * 0.5) * (double)(k - image_width * 0.5);

            //データが存在する全フレーム繰り返し
            std::map<int, std::vector<cv::Point2d>> undistorted_corner_dict;
            for (const auto &el : corner_dict)
            {
                // undistorted_corner_dictを生成
                std::vector<cv::Point2d> dst;
                manager.getUndistortUnrollingChessBoardPoints(el.first / capture->get(cv::CAP_PROP_FPS) + time_offset, el.second, dst, rolling_shutter_parameter);
                undistorted_corner_dict[el.first] = dst;
            }

            // Minimum correration
            if (0)
            {
                // setEstimatedAngularVelocity
                estimated_angular_velocity = manager.estimateAngularVelocity(undistorted_corner_dict, world_points, confidence);
                manager.setEstimatedAngularVelocity(estimated_angular_velocity, confidence, capture->get(cv::CAP_PROP_FPS));

                // 相関を取る
                correlation = manager.estimate();

                // 相関をmatに記録する
                optimize_result_mat.at<double>(i, k) = correlation.minCoeff(); //i:time offset, 行, u,y //k:rolling shutter_parameter, 列, v, x
            }
            else
            // Reprojection Error
            {
                std::vector<cv::Mat> rvecs;
                std::vector<cv::Mat> tvecs;
                std::vector<std::vector<cv::Point3d>> vec_world_points;
                std::vector<std::vector<cv::Point2d>> vec_image_points;

                cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << camera_info->fx_, 0, camera_info->cx_, 0, camera_info->fy_, camera_info->cy_, 0, 0, 1);
                cv::Mat dist_coeffs = (cv::Mat_<double>(1, 4) << camera_info->k1_, camera_info->k2_, camera_info->p1_, camera_info->p2_);
                std::vector<double> per_image_errors;
                for (auto &el : undistorted_corner_dict)
                {
                    cv::Mat rvec;
                    cv::Mat tvec;

                    cv::solvePnP(world_points, el.second, camera_matrix, dist_coeffs, rvec, tvec);
                    rvecs.push_back(rvec);
                    tvecs.push_back(tvec);
                    vec_world_points.push_back(world_points);
                    vec_image_points.push_back(el.second);
                }
                optimize_result_mat.at<double>(i, k) = (double)manager.computeReprojectionErrors(vec_world_points, vec_image_points, rvecs, tvecs, camera_matrix, dist_coeffs, per_image_errors);
            }
        }
    }
    cv::normalize(optimize_result_mat, optimize_result_mat, 255, 0, cv::NORM_MINMAX);
    cv::namedWindow("optimized result", cv::WINDOW_NORMAL);
    cv::Mat optimize_result_mat_to_show;
    optimize_result_mat.convertTo(optimize_result_mat_to_show, CV_8UC1);
    cv::imshow("optimized result", optimize_result_mat_to_show);
    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;
    cv::minMaxLoc(optimize_result_mat, &minVal, &maxVal, &minLoc, &maxLoc);
    double optimal_time_offset = 1.0 / capture->get(cv::CAP_PROP_FPS) / ((double)image_width * 0.5) * (double)(minLoc.y - image_width * 0.5);
    std::cout << "Optimal time offset is " << optimal_time_offset
              << "." << std::endl;
    double optimal_rolling_shutter_coefficient = 1.0 / capture->get(cv::CAP_PROP_FPS) / ((double)image_width * 0.5) * (double)(minLoc.x - image_width * 0.5);
    std::cout << "Optimal rolling shutter coefficient is " << optimal_rolling_shutter_coefficient
              << "." << std::endl
              << std::flush;

    cv::waitKey(0);
    //ここにgetUndistortUnrollingChessBoardPointsで、Rolling shutter coefficientを変化させながら、チェスボードパターンがどう変化するかを示した画像を表示したい

    //難しいことしないでまずは普通にチェスボードを表示してみる。
    //画面がでかすぎるので半分に縮小して表示

    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    cv::Mat color_image;
    while (1)
    {
        int frame = capture->get(cv::CAP_PROP_POS_FRAMES);
        (*capture) >> color_image;
        if (color_image.empty())
        {
            break;
        }

        cv::resize(color_image, color_image, cv::Size(), 0.5, 0.5);
        if (corner_dict.count(frame))
        {
            // Shrink image and corners since image is too large
            std::vector<cv::Point2d> shrinked, dst;
            for (auto &el : corner_dict[frame])
            {
                shrinked.push_back(el * 0.5);
            }
            cv::drawChessboardCorners(color_image, PatternSize, shrinked, false);

            // Generate unrolled and undistorted corners
            manager.getUndistortUnrollingChessBoardPoints((double)frame / capture->get(cv::CAP_PROP_FPS) + optimal_time_offset, corner_dict[frame], dst, optimal_rolling_shutter_coefficient);
            shrinked.clear();
            for (auto &el : dst)
            {
                shrinked.push_back(el * 0.5);
            }
            cv::drawChessboardCorners(color_image, PatternSize, shrinked, true);
        }

        cv::imshow("image", color_image);
        // if (!debug_speedup)
        // {
        //     if ('q' == (char)cv::waitKey(1))
        //     {
        //         break;
        //     }
        // }
        // else
        // {
        if ('q' == (char)cv::waitKey(0))
        {
            break;
        }
        // }
    }
    cv::destroyAllWindows();
    return 0;
}
