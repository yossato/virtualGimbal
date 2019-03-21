#include <stdio.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "mINIRead.hpp"
int main(int argc, char **argv)
{
    //引数の確認
    char *videoPass = NULL;
    char *cameraName = NULL;
    char *lensName = NULL;
    int rotation_type;
    int opt;
    //    Eigen::Quaterniond camera_rotation;

    constexpr double S = pow(2, -0.5);
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

    while ((opt = getopt(argc, argv, "i:c:l:r:")) != -1)
    {
        switch (opt)
        {
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
    std::vector<std::vector<cv::Point2f>> imagePoints;                                 // チェッカー交点座標を格納するベクトルのベクトル インデックスの並びは[撮影画像番号][点のIndex]
    std::vector<std::vector<cv::Point3f>> worldPoints((int)Dcbp.NumberOfCaptureImage); // チェッカー交点座標と対応する世界座標の値を格納する行列
    // 世界座標を決める
    for (int i = 0; i < Dcbp.NumberOfCaptureImage; i++)
    {
        for (int j = 0; j < PatternSize.area(); j++)
        { //チェッカーボードの交点座標を記録　
            worldPoints[i].push_back(cv::Point3f(static_cast<float>(j % PatternSize.width * Dcbp.SizeOfQuadsX_mm),
                                                 static_cast<float>(j / PatternSize.width * Dcbp.SizeOfQuadsY_mm),
                                                 0.0));
        }
    }

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
        }
    }

    for (auto &el : corner_dict)
    {
        std::cout << el.first << std::endl;
    }

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