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
std::string getVideoSize(const char *videoName)
{
    std::shared_ptr<cv::VideoCapture> Capture = std::make_shared<cv::VideoCapture>(videoName); //動画をオープン
    assert(Capture->isOpened());
    std::string videoSize = std::to_string((int)Capture->get(cv::CAP_PROP_FRAME_WIDTH)) + std::string("x") + std::to_string((int)Capture->get(cv::CAP_PROP_FRAME_HEIGHT));
    return videoSize;
}

int main(int argc, char **argv)
{
    //引数の確認
    char *videoPass = NULL;
    char *cameraName = NULL;
    char *lensName = NULL;
    char *jsonPass = NULL;
    int rotation_type;
    int opt;
    //    Eigen::Quaterniond camera_rotation;

    const double S = pow(2, -0.5);
    constexpr double F = 0.5;
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> vector_sd_card_rotation = {
        Eigen::Quaterniond(F, F, -F, F), //0
        Eigen::Quaterniond(F, -F, F, F),
        Eigen::Quaterniond(S, 0, 0, S),
        Eigen::Quaterniond(0, -S, S, 0),
        Eigen::Quaterniond(S, 0, -S, 0),
        Eigen::Quaterniond(S, 0, S, 0), //5
        Eigen::Quaterniond(1, 0, 0, 0),
        Eigen::Quaterniond(0, 0, 1, 0),
        Eigen::Quaterniond(F, F, F, -F),
        Eigen::Quaterniond(F, -F, -F, -F),
        Eigen::Quaterniond(0, S, S, 0), //10
        Eigen::Quaterniond(S, 0, 0, -S),
        Eigen::Quaterniond(0, 0, -S, S),
        Eigen::Quaterniond(S, -S, 0, 0),
        Eigen::Quaterniond(F, -F, -F, F),
        Eigen::Quaterniond(F, -F, F, -F) //15
    };

    Eigen::Quaterniond sd_card_rotation;

    while ((opt = getopt(argc, argv, "j:i:c:l:r:")) != -1)
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
        case 'r':
            rotation_type = std::atoi(optarg);
            sd_card_rotation = vector_sd_card_rotation[rotation_type];
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
    manager.setMeasuredAngularVelocity(jsonPass);
    shared_ptr<CameraInformation> camera_info(new CameraInformationJsonParser(cameraName, lensName, getVideoSize(videoPass).c_str()));
    manager.setVideoParam(videoPass, camera_info);
    manager.setRotation(jsonPass, *camera_info);

    //    manager.setEstimatedAngularVelocity(videoPass,camera_info);

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
    std::vector<std::vector<cv::Point2f>> imagePoints; // チェッカー交点座標を格納するベクトルのベクトル インデックスの並びは[撮影画像番号][点のIndex]

    //キャリブレーションの準備ここまで
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
            if (cv::findChessboardCorners(gray_image, PatternSize, acquired_image_points, cv::CALIB_CB_FAST_CHECK))
            {
                cv::cornerSubPix(gray_image, acquired_image_points, cv::Size(11, 11), cv::Size(-1, -1), criteria);
                corner_dict[i] = acquired_image_points;
            }
            printf("%d/%d\r", i, e);
            std::cout << std::flush;

            // Speed up for debug
            if (i == 100)
            break;
        }
    }

    cv::Mat CameraMatrix = (cv::Mat_<float>(3, 3) << camera_info->fx_, 0, camera_info->cx_, 0, camera_info->fy_, camera_info->cy_, 0, 0, 1);
    cv::Mat DistCoeffs = (cv::Mat_<float>(1, 4) << camera_info->k1_, camera_info->k2_, camera_info->p1_, camera_info->p2_);
    std::map<int, cv::Mat> RotationVector;
    std::map<int, cv::Mat> TranslationVector;

    std::vector<cv::Point3f> world_points; // チェッカー交点座標と対応する世界座標の値を格納する行列
    // 世界座標を決める
    for (int j = 0; j < PatternSize.area(); j++)
    { //チェッカーボードの交点座標を記録
        world_points.push_back(cv::Point3f(static_cast<float>(j % PatternSize.width * Dcbp.SizeOfQuadsX_mm),
                                           static_cast<float>(j / PatternSize.width * Dcbp.SizeOfQuadsY_mm),
                                           0.0));
    }

    Eigen::VectorXd confidence = Eigen::VectorXd::Zero(capture->get(cv::CAP_PROP_FRAME_COUNT));
    Eigen::MatrixXd estimated_angular_velocity = Eigen::MatrixXd::Zero(capture->get(cv::CAP_PROP_FRAME_COUNT), 3);

    for (const auto &el : corner_dict)
    {
        cv::solvePnP(world_points, el.second, CameraMatrix, DistCoeffs, RotationVector[el.first], TranslationVector[el.first]);
        // printf("%d,%f,%f,%f ",el.first,RotationVector[el.first].at<float>(0,0),RotationVector[el.first].at<float>(1,0),RotationVector[el.first].at<float>(2,0));
        Eigen::Quaterniond rotation_quaternion = vsp::Vector2Quaternion<double>(Eigen::Vector3d(RotationVector[el.first].at<float>(0, 0), RotationVector[el.first].at<float>(1, 0), RotationVector[el.first].at<float>(2, 0)));
        printf("%d,%f,%f,%f,%f,", el.first, rotation_quaternion.x(), rotation_quaternion.y(), rotation_quaternion.z(), rotation_quaternion.w());
        if (0 != RotationVector.count(el.first - 1))
        {
            Eigen::Quaterniond rotation_quaternion_previous = vsp::Vector2Quaternion<double>(Eigen::Vector3d(RotationVector[el.first - 1].at<float>(0, 0), RotationVector[el.first - 1].at<float>(1, 0), RotationVector[el.first - 1].at<float>(2, 0)));
            // cv::Mat diff = RotationVector[el.first]-RotationVector[el.first-1];
            Eigen::Quaterniond diff = rotation_quaternion.conjugate() * rotation_quaternion_previous;
            // printf("%f,%f,%f\n",diff.at<float>(0,0),diff.at<float>(1,0),diff.at<float>(2,0));
            printf("%f,%f,%f,%f\n", diff.x(), diff.y(), diff.z(), diff.w());
            estimated_angular_velocity.row(el.first) = vsp::Quaternion2Vector(diff).transpose();
            confidence(el.first) = 1.0;
        }
        else
        {
            printf("0,0,0\n");
        }
    }
    std::cout << std::flush;

    estimated_angular_velocity = estimated_angular_velocity * capture->get(cv::CAP_PROP_FPS);
    manager.setEstimatedAngularVelocity(estimated_angular_velocity, confidence, capture->get(cv::CAP_PROP_FPS));

    // std::cout << "estimated_angular_velocity:" << std::endl;
    // std::cout << estimated_angular_velocity << std::endl;
    // std::cout << "confidence" << std::endl;
    // std::cout << confidence.block(0,0,100,1) << std::endl;

    Eigen::MatrixXd correlation = manager.estimate();
    std::vector<string> legends_angular_velocity = {"c"};
    vgp::plot(correlation, "correlation",legends_angular_velocity);

    return 0;

    double Tav = 1. / readSamplingRateFromJson(jsonPass); //Sampling period of angular velocity
    double Tvideo = 1.0 / capture->get(cv::CAP_PROP_FPS);

    //    vsp v2(9,9,*camera_info,1.0,angular_velocity_from_csv,
    //               Tvideo,
    //               Tav,
    //               min_position + subframeOffset,
    //               (int32_t)(capture->get(cv::CAP_PROP_FRAME_COUNT)),
    //               199);

    int c;
    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);

    while ('q' != (char)cv::waitKey(1))
    { //信号が来るまで
        //ここからキャリブレーションの本体
        static int init = 0;

        (*capture) >> color_image;
        cv::imshow("image", color_image);
    }
    return 0;
}
