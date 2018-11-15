#include <stdio.h>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

// Include standard headers
#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>

#include "calcShift.hpp"
#include <chrono>
#include "mINIRead.hpp"
using namespace std;



int main(int argc, char** argv){
    //引数の確認
    char *videoPass = NULL;
    char *csvPass = NULL;
    int opt;
    while((opt = getopt(argc, argv, "i:c:")) != -1){
        switch (opt) {
        case 'i':
            videoPass = optarg;
            break;
        case 'c':
            csvPass = optarg;
            break;
        default :
            printf("Use options. -i videofilepass -c csvfilepass.\r\n");
            return 1;
        }
    }

    //動画からオプティカルフローを計算する
    auto t1 = std::chrono::system_clock::now() ;
    std::vector<cv::Vec3d> opticShift = CalcShiftFromVideo(videoPass,1000);//ビデオからオプティカルフローを用いてシフト量を算出
    auto t2 = std::chrono::system_clock::now() ;
    // 処理の経過時間
    auto elapsed = t2 - t1 ;
    std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << " ms\n";

    cv::VideoCapture Capture(videoPass);//動画をオープン
    assert(Capture.isOpened());
    cv::Size imageSize = cv::Size(Capture.get(cv::CAP_PROP_FRAME_WIDTH),Capture.get(cv::CAP_PROP_FRAME_HEIGHT));//解像度を読む
    double Tvideo = 1.0/Capture.get(cv::CAP_PROP_FPS);
    std::cout << "resolution" << imageSize << std::endl;
    std::cout << "samplingPeriod" << Tvideo << std::endl;

    //内部パラメータを読み込み
    cv::Mat matIntrinsic;
    ReadIntrinsicsParams("intrinsic.txt",matIntrinsic);
    std::cout << "Camera matrix:\n" << matIntrinsic << "\n" <<  std::endl;
    double fx = matIntrinsic.at<double>(0,0);
    double fy = matIntrinsic.at<double>(1,1);
    double cx = matIntrinsic.at<double>(0,2);
    double cy = matIntrinsic.at<double>(1,2);


    //歪パラメータの読み込み
    cv::Mat matDist;
    ReadDistortionParams("distortion.txt",matDist);
    std::cout << "Distortion Coeff:\n" << matDist << "\n" << std::endl;

    cv::Mat img;

    //試しに先に進む
    Capture.set(cv::CAP_PROP_POS_FRAMES,1000);

    //動画の読み込み
    Capture >> img;

    //角速度データを読み込み
    std::vector<cv::Vec3d> angularVelocityIn60Hz;
    ReadCSV(angularVelocityIn60Hz,csvPass);

    //軸の定義方向の入れ替え
    //TODO:将来的に、CSVファイルに順番を揃えて、ゲインも揃えた値を書き込んでおくべき
    for(auto &el:angularVelocityIn60Hz){
            auto temp = el;
            el[0] = temp[1]/16.4*M_PI/180.0;//16.4はジャイロセンサが感度[LSB/(degree/s)]で2000[degrees/second]の時のもの。ジャイロセンサの種類や感度を変更した時は値を変更する;
            el[1] = temp[0]/16.4*M_PI/180.0;
            el[2] = -temp[2]/16.4*M_PI/180.0;
    }
    double Tav = 1/60.0;//Sampling period of angular velocity

    //動画のサンプリング周期に合わせて、角速度を得られるようにする関数を定義
    //線形補間
    auto angularVelocity = [&angularVelocityIn60Hz, Tvideo, Tav](uint32_t frame){
        double dframe = frame * Tvideo/Tav;
        int i = floor(dframe);
        double decimalPart = dframe - (double)i;
        return angularVelocityIn60Hz[i]*(1.0-decimalPart)+angularVelocityIn60Hz[i+1]*decimalPart;
    };

    cout << "angular Velocity" << endl;
    for(int i=0;i<1000;i++){
        cout << angularVelocity(i) << endl;
    }



    //動画のオプティカルフローと内部パラメータと解像度から角速度推定値を計算
    vector<cv::Vec3d> estimatedAngularVelocity;
    cout << "estimated AngularVelocity" << endl;
    for(auto el:opticShift){
        estimatedAngularVelocity.push_back(cv::Vec3d(-atan(el[1]/fy),atan(el[0]/fx),el[2])/Tvideo*-1);
        cout << estimatedAngularVelocity.back() << endl;
    }

    int32_t lengthDiff = angularVelocityIn60Hz.size() * Tvideo / Tav - estimatedAngularVelocity.size();
    cout << "lengthDiff:" << lengthDiff << endl;
    vector<double> correlationCoefficients(lengthDiff);
    double minCC = DBL_MAX;
    for(int32_t offset = 0; offset < lengthDiff; offset++){
        double sum = 0.0;
        for(int32_t i=0; i<estimatedAngularVelocity.size();i++){
            sum +=   abs(angularVelocity(i+offset)[0]-estimatedAngularVelocity[i][0])
                   + abs(angularVelocity(i+offset)[1]-estimatedAngularVelocity[i][1])
                   + abs(angularVelocity(i+offset)[2]-estimatedAngularVelocity[i][2]);
//            if(sum > minCC){
//                break;
//            }
        }
        if(sum < minCC){
            minCC = sum;
        }
        correlationCoefficients[offset] = sum;
    }

    cout << "correlationCoefficients" << endl;
    for(auto el:correlationCoefficients) cout << el << endl;

	//最小となる要素を取得
	int32_t minPosition = std::distance(correlationCoefficients.begin(),min_element(correlationCoefficients.begin(),correlationCoefficients.end()));
	//最小値サブピクセル推定
	double subframeOffset;
	if(minPosition == 0){	//位置が最初のフレームで一致している場合
		subframeOffset = 0.0;
	}else if(minPosition == (lengthDiff-1)){//末尾
		subframeOffset = (double)(lengthDiff -1);
	}else{					//その他
		if(correlationCoefficients[minPosition-1] >= correlationCoefficients[minPosition+1]){
            subframeOffset = (correlationCoefficients[minPosition-1] - correlationCoefficients[minPosition+1])/(2*correlationCoefficients[minPosition-1]-2*correlationCoefficients[minPosition]);
		}else{
            subframeOffset = -(correlationCoefficients[minPosition+1]-correlationCoefficients[minPosition-1])/(2*correlationCoefficients[minPosition+1]-2*correlationCoefficients[minPosition]);
		}
	}

    cout << "minPosition" << minPosition << endl;
    cout << "subframe minposition :" << minPosition+subframeOffset << endl;

    auto angularVelocity_double = [&angularVelocityIn60Hz, Tvideo, Tav](double frame){
        double dframe = frame * Tvideo/Tav;
        int i = floor(dframe);
        double decimalPart = dframe - (double)i;
        return angularVelocityIn60Hz[i]*(1.0-decimalPart)+angularVelocityIn60Hz[i+1]*decimalPart;
    };
    for(double d=-0.5;d<=0.5;d+=0.01)
    {

        double sum = 0.0;
        for(int32_t i=0; i<estimatedAngularVelocity.size();i++){
            sum +=   abs(angularVelocity_double(i+minPosition+subframeOffset+d)[0]-estimatedAngularVelocity[i][0])
                   + abs(angularVelocity_double(i+minPosition+subframeOffset+d)[1]-estimatedAngularVelocity[i][1])
                   + abs(angularVelocity_double(i+minPosition+subframeOffset+d)[2]-estimatedAngularVelocity[i][2]);
//            if(sum > minCC){
//                break;
//            }
        }
        cout << "position"<<minPosition+subframeOffset+d<<" minimum correlationCoefficients:" << sum << endl;
    }

//    for(auto &el:vecw) std::cout << el << std::endl;
}
