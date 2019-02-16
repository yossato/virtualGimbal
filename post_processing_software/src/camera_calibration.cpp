#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include "mINIRead.hpp"
#include <thread>
#include <future>
#include <unistd.h>
#include "time.h"
#include <semaphore.h>
#include <sys/time.h>
#include <functional>
#include "camera_information.h"
#include "json_tools.hpp"
int nCount=0;
volatile int _Quit = 0;//終了するかどうか。0以外で終了

/**
* @brief ステレオキャリブレーション結果を保持する構造体
**/
typedef struct{
	int imagePointsFrame0;		//!<0個目のカメラのimagePointsフレーム。imagePointsに保存された時系列データのどこに目的データがあるのか、インデックスを保存しておく。
	int imagePointsFrameN;		//!<N個目のカメラのimagePointsフレーム
	int CheckerBoard;				//!<計測されたチェッカーボード番号。vecPatternSizeやworldPointsのインデックス。
}strValidPointPair;


/**
* @brief 入力画像からチェッカーボードを探し、結果の座標を返す
* @param [in] Finish 計算が終わったかどうか。0:未完　1:完了
*
**/
std::vector<cv::Point2f> findChessboardCornersInMultithread(cv::Mat &image, cv::Size patternSize, int &Finish, int flags = cv::CALIB_CB_FAST_CHECK)
{
	Finish = 0;//コーナ検索未完
	std::vector<cv::Point2f>		tempImagePoints;
	static int n = 0;//この関数は何回呼ばれた？
	int cn = n++;
	cv::Mat GrayImage;
    cv::TermCriteria						criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 20, 0.001);
	//std::cout << "Finding corners from Image " << cn;
	if (cv::findChessboardCorners(image, patternSize, tempImagePoints, flags)) {
		//std::cout << " ... All corners are found at Image " << cn << "." << std::endl;
		// 検出点をカラー画像のほうへ、描画する
		if ((image.channels() == 3) || (image.channels() == 4)){
            cv::cvtColor(image, GrayImage, cv::COLOR_RGB2GRAY);
			cv::cornerSubPix(GrayImage, tempImagePoints, cv::Size(11, 11), cv::Size(-1, -1), criteria);
		}
		else{
			cv::cornerSubPix(image, tempImagePoints, cv::Size(11, 11), cv::Size(-1, -1), criteria);
		}
		
	}
	else{
		//std::cout << " ... No corner is found at Image" << cn << "." << std::endl;
	}
	Finish = 1;//コーナ検索完了

	//検出できても、できていなくても、ポイントを返す。
	return tempImagePoints;
}

void ConvertMat8to16(cv::Mat &src, cv::Mat  &dst){
	//IR画像を16bitから8bitへ直す
	cv::Mat temp(src.rows, src.cols, CV_8U);
	auto pdst = temp.data;
	for (auto t = (unsigned short*)src.data, e = (unsigned short*)src.data + src.size().area(); t != e;){
		const int ShiftBits = 6;
		const int blackLevel = (int)(pow(2.0, (double)ShiftBits + 8.0)*0.35);
		const int thres = (int)pow(2.0, (double)ShiftBits + 8.0) + blackLevel;
		if (*t >= thres){
			*pdst++ = 255;
			t++;
			continue;
		}
		else if (*t <= blackLevel){
			*pdst++ = 0;
			t++;
			continue;
		}
		*pdst++ = (*t++ - blackLevel) >> ShiftBits;	//8bitへ変換 2^8で割ると信号が小さくなりすぎるので、2^6で割る
	}
	//tempf = CamImage[1] / 256;	
	dst = temp;
	//cv::threshold(temp, CamImage[1], 128, 255, cv::THRESH_TOZERO);

}

static const char *WindowName[] = { "Color", "IR"};//window name

static int Captured = 0;
		int width = 640;
		int height = 480;
		static cv::Mat grayImg = cv::Mat::zeros(height,width,CV_8U);	//グレー画像の準備


/**
 * @brief main関数。RaspiCamのキャリブレーションを実行。
 * 
 * 
 **/
int main(int argc, char** argv){
    //引数の確認
    char *videoPass = NULL;
    char *cameraName = NULL;
    char *lensName = NULL;
    int rotation_type;
    int opt;
//    Eigen::Quaterniond camera_rotation;

    constexpr double S = pow(2,-0.5);
    constexpr double F = 0.5;
    std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond>> vector_sd_card_rotation = {
        Eigen::Quaterniond( F, F,-F, F),//0
        Eigen::Quaterniond( F,-F, F, F),
        Eigen::Quaterniond( S, 0, 0, S),
        Eigen::Quaterniond( 0,-S, S, 0),
        Eigen::Quaterniond( S, 0,-S, 0),
        Eigen::Quaterniond( S, 0, S, 0),//5
        Eigen::Quaterniond( 1, 0, 0, 0),
        Eigen::Quaterniond( 0, 0, 1, 0),
        Eigen::Quaterniond( F, F, F,-F),
        Eigen::Quaterniond( F,-F,-F,-F),
        Eigen::Quaterniond( 0, S, S, 0),//10
        Eigen::Quaterniond( S, 0, 0,-S),
        Eigen::Quaterniond( 0, 0,-S, S),
        Eigen::Quaterniond( S,-S, 0, 0),
        Eigen::Quaterniond( F,-F,-F, F),
        Eigen::Quaterniond( F,-F, F,-F) //15
    };

    Eigen::Quaterniond sd_card_rotation;

    while((opt = getopt(argc, argv, "i:c:l:r:")) != -1){
        switch (opt) {
        case 'i':       //input video file pass
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
        default :
//            printf(     "virtualGimbal\r\n"
//                        "Hyper fast video stabilizer\r\n\r\n"
//                        "usage: virtualGimbal [-i video] [-f angularVelocity] [[output option] -o] [options]\r\n"
//                        );
            return 1;
        }
    }

    if(0){
        CameraInformationJsonParser cameraInfo;

        cameraInfo.camera_name_ = cameraName;
        cameraInfo.lens_name_ = lensName;
        cameraInfo.sd_card_rotation_ = sd_card_rotation;
		cameraInfo.width_ = 2;
		cameraInfo.p1_ = 2.0;

        cameraInfo.writeCameraInformationJson("camera_descriptions/cameras_testtest.json");
        return 0;
    }

    // TODO: Show chess board pattern on a screen.

	//動画読み込み準備
//	char *filename;
//	char defaultName[] = "";
//	if(argc==1){
//		printf("引数としてチェスボードを撮影した動画を指定してください。\nプログラムを終了します。\n");
//		return 0;
//	}
//	filename = new char[strlen(argv[1])];
//	strcpy(filename, (const char *)argv[1]);//ファイル名をコピー
    cv::VideoCapture Capture(videoPass);//動画をオープン
	if(!Capture.isOpened()){//VideoCaptureは初期化に成功しているはず。
		printf("動画ファイルを開くことができませんでした。\nプログラムを終了します。\n");
		abort();
	}


	//動画読み込み準備ここまで
	
	//チェッカーボードの設定iniファイルを読み込み
    char path[512];
    getcwd(path,sizeof(path));
    std::string path_string(path);
    path_string = path_string + "/chess_board_settings.ini";
    if (ReadINIs(path_string.c_str(), INICheckerBoardParamNum, INICheckerBoardValueNames, Dcbp) != 0){
	//	system("pause");	//ユーザのキーボード入力を待機
		return 1;
	}
	
	//キャリブレーションの準備
	cv::Size	PatternSize = cv::Size((int)Dcbp.NumberOfInnerCorners.X, (int)Dcbp.NumberOfInnerCorners.Y);
	std::vector<std::vector<cv::Point2f>>	imagePoints;		// チェッカー交点座標を格納するベクトルのベクトル インデックスの並びは[撮影画像番号][点のIndex]
	std::vector<std::vector<cv::Point3f>>	worldPoints((int)Dcbp.NumberOfCaptureImage);		// チェッカー交点座標と対応する世界座標の値を格納する行列
	// 世界座標を決める
	for(int i=0;i<Dcbp.NumberOfCaptureImage;i++){
		for (int j = 0; j < PatternSize.area(); j++) {//チェッカーボードの交点座標を記録　
			worldPoints[i].push_back(cv::Point3f(static_cast<float>(j % PatternSize.width * Dcbp.SizeOfQuadsX_mm),
				static_cast<float>(j / PatternSize.width * Dcbp.SizeOfQuadsY_mm),
				0.0));
		}
	}

	//キャリブレーションの準備ここまで

	
	time_t timer_begin,timer_end;
	double secondsElapsed;
	


	int c;
    cv::namedWindow(WindowName[0],cv::WINDOW_AUTOSIZE);
	cv::Mat colorImg;
	while(!_Quit){	//信号が来るまで
		//ここからキャリブレーションの本体
		static int init = 0;

		Capture >> colorImg;
		
        // Quit a loop if a capture failed. This is end of a input video frame.
	    if (colorImg.empty()){	    
		    std::cout << "Video finished. Now Calicurating parameters... please wait...\r\n" << std::endl;
            cv::destroyWindow(WindowName[0]);
		    break;
		}
		
        cv::cvtColor(colorImg,grayImg,cv::COLOR_RGB2GRAY);
		    
		//~ if(Captured){
		cv::imshow(WindowName[0],grayImg);//生画像を表示
		Captured = 0;
		nCount++; 
		if('q' ==(char)cv::waitKey(1)){
			_Quit = 1;//qをおしたらプログラム終了
		}
		int Finish = 0;
#if 0	//マルチスレッドプログラム
		std::future<std::vector<cv::Point2f>> cbResults;
		cbResults = std::async(std::launch::async, findChessboardCornersInMultithread, std::ref(grayImg), PatternSize, std::ref(Finish), CV_CALIB_CB_FAST_CHECK);
		while (Finish == 0){//asyncでチェッカーボードを検出している間は画面表示し続ける
			if(Captured){
				cv::imshow(WindowName[0],grayImg);//生画像を表示
				Captured = 0;
				if('q' ==(char)cv::waitKey(1)){
					_Quit = 1;//qをおしたらプログラム終了
				}
			}
		}
		auto result = cbResults.get();
#else      //シングルスレッド
        auto result = findChessboardCornersInMultithread(grayImg, PatternSize, Finish, cv::CALIB_CB_FAST_CHECK);
#endif
        if (result.size()==PatternSize.area()) {// If a result found
			static int CapturedTimes = 0;
			static cv::Mat colorImg = cv::Mat::zeros(height,width,CV_8UC3);	//画面表示用のカラー画像の準備
            cv::cvtColor(grayImg,colorImg,cv::COLOR_GRAY2RGB);
			imagePoints.push_back(result);//asyncで計算していた結果を受け取る
			cv::drawChessboardCorners(colorImg, PatternSize, (cv::Mat)imagePoints.back(), true);
			cv::imshow(WindowName[0], colorImg);//IR画像を表示
			cv::waitKey(1);
			
			printf("All corners are found : %d\r\n",CapturedTimes++);
		}


	}
	
	//規定枚数まで結果を縮小
	std::vector<std::vector<cv::Point2f>> shrinkImagePoints;
	for(int i=0;i<Dcbp.NumberOfCaptureImage;++i){
		shrinkImagePoints.push_back(imagePoints[(int)round((double)(imagePoints.size())/(double)(Dcbp.NumberOfCaptureImage)*(double)i)]);
	}
	
    printf("shrink size:%d\n",(int)shrinkImagePoints.size());
	imagePoints = shrinkImagePoints;//入れ替え
	
	//ここからカメラパラメータを求める
//	cv::Mat R, T, E, F;
	cv::Mat CameraMatrix;
	cv::Mat DistCoeffs;
	std::vector<cv::Mat> RotationVector;
	std::vector<cv::Mat> TranslationVector;

	//RGBカメラのキャリブレーション実施
	if ((unsigned int)Dcbp.NumberOfCaptureImage <= imagePoints.size()){
		cv::calibrateCamera(worldPoints, imagePoints, grayImg.size(), CameraMatrix, DistCoeffs, RotationVector, TranslationVector);
	
		std::cout << "*************************************************************************" << std::endl;
		std::cout << "Color Camera parameters have been estimated" << std::endl << std::endl;
		std::cout << "CameraMatrix:" << std::endl;
		std::cout << CameraMatrix << std::endl;
		std::cout << "Distortion Coefficients:" << std::endl;
		std::cout << DistCoeffs << std::endl;
		std::cout << "*************************************************************************" << std::endl;

	
        CameraInformationJsonParser cameraInfo;
        cameraInfo.fx_ = CameraMatrix.at<double>(0,0);
        cameraInfo.fy_ = CameraMatrix.at<double>(1,1);
        cameraInfo.cx_ = CameraMatrix.at<double>(0,2);
        cameraInfo.cy_ = CameraMatrix.at<double>(1,2);
        cameraInfo.k1_ = DistCoeffs.at<double>(0,0);
        cameraInfo.k2_ = DistCoeffs.at<double>(0,1);
        cameraInfo.p1_ = DistCoeffs.at<double>(0,2);
        cameraInfo.p2_ = DistCoeffs.at<double>(0,3);
        cameraInfo.inverse_k1_ = 0.;
        cameraInfo.inverse_k2_ = 0.;
        cameraInfo.inverse_p1_ = 0.;
        cameraInfo.inverse_p2_ = 0.;
        cameraInfo.rolling_shutter_coefficient_ = 0.; //This calibration does not estimate rolling shutter coefficient.
        cameraInfo.width_ = Capture.get(cv::CAP_PROP_FRAME_WIDTH);
        cameraInfo.height_ = Capture.get(cv::CAP_PROP_FRAME_HEIGHT);

        cameraInfo.camera_name_ = cameraName;
        cameraInfo.lens_name_ = lensName;
        cameraInfo.sd_card_rotation_ = sd_card_rotation;

        cameraInfo.writeCameraInformationJson();

        return 0;


//		std::cout << "キャリブレーションが完了しました。\r\n結果をファイルに保存しますか？ Yで結果を保存 Nで破棄\nColorウィンドウにキーボードで入力してください。" << std::endl;



//		while (1){
			
//			//char result[256];
//			//std::cin >> result;
//			char key = cv::waitKey(1);
//			if ((key == 'Y') || (key == 'y')){	//ファイル保存

//				try{
//					std::cout << "y" << std::endl;
//					std::ofstream ofs;
//					//カラーカメラの固有パラメータ
//					ofs.open("intrinsic.txt");
//					for (int i = 0; i < 3; i++){
//						ofs << CameraMatrix.at<double>(i, 0) << " " << CameraMatrix.at<double>(i, 1) << " " << CameraMatrix.at<double>(i, 2) << std::endl;
//					}
//					ofs.close();

//					//カメラのレンズひずみパラメータ
//					ofs.open("distortion.txt");
//					ofs << DistCoeffs.at<double>(0, 0) << " " << DistCoeffs.at<double>(0, 1) << " " << DistCoeffs.at<double>(0, 2) << " " << DistCoeffs.at<double>(0, 3) << std::endl;
//					ofs.close();
//				}
//				catch (...){
//					std::cout << "キャリブレーション結果の保存に失敗しました。iniファイルが読み取り専用になっていないか、違うソフトがiniファイルを使用中でないか確認してください。" << std::endl;
//				}
//				std::cout << "カラーカメラの固有パラメータをファイルに保存しました。" << std::endl;
//				char filenamestxt[] = "保存したファイル名は以下の通りです。\r\n"
//					"intrinsic.txt\r\n"
//					"distortion.txt\r\n"
//					"プログラムを終了します...";
//					printf("%s", filenamestxt);
//				break;
//			}
//			else if ((key == 'N') || (key == 'n')){	//破棄
//				std::cout << "n" << std::endl;
//				std::cout << "結果を破棄しました。" << std::endl;
//				break;
//			}
//		}

		
    }else{
        std::cout << "Error: Available frame are not found.\nPlease use another video." << std::endl;
    }
		
	return 0;	
}
 
