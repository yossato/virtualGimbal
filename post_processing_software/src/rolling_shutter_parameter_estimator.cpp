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

// std::map<int, std::vector<cv::Point2f>> getCornerDictionary(std::shared_ptr<cv::VideoCapture> capture, cv::Size &pattern_size, bool debug_speedup, bool Verbose);

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
    std::vector<std::vector<cv::Point2f>> imagePoints; // チェッカー交点座標を格納するベクトルのベクトル インデックスの並びは[撮影画像番号][点のIndex]

    //キャリブレーションの準備ここまで
    std::map<int, std::vector<cv::Point2f>> corner_dict = manager.getCornerDictionary(PatternSize, debug_speedup, true);

    std::vector<cv::Point3f> world_points; // チェッカー交点座標と対応する世界座標の値を格納する行列
    // 世界座標を決める
    for (int j = 0; j < PatternSize.area(); j++)
    { //チェッカーボードの交点座標を記録
        world_points.push_back(cv::Point3f(static_cast<float>(j % PatternSize.width * Dcbp.SizeOfQuadsX_mm),
                                           static_cast<float>(j / PatternSize.width * Dcbp.SizeOfQuadsY_mm),
                                           0.0));
    }

    Eigen::VectorXd confidence;
    Eigen::MatrixXd estimated_angular_velocity = manager.estimateAngularVelocity(corner_dict, world_points,confidence);
    std::cout << estimated_angular_velocity << std::endl << std::flush;
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

    return 0;

    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);

    cv::Mat color_image;
    while ('q' != (char)cv::waitKey(1))
    { //信号が来るまで
        //ここからキャリブレーションの本体
        // static int init = 0;

        (*capture) >> color_image;
        cv::imshow("image", color_image);
    }
    return 0;
}



