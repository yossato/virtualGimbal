#include <stdio.h>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <iomanip>
//Quartanion
#include <boost/math/quaternion.hpp>
#include "stabilize.h"
#include "distortion.h"
using namespace boost::math;

// Include standard headers
#include <stdio.h>
#include <stdlib.h>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <glfw3.h>
//GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include <common/shader.hpp>
#include <common/texture.hpp>
#include <common/controls.hpp>

#include "calcShift.hpp"
#include <chrono>

#include "mINIRead.hpp"

//Numetric optimization
#include "levenbergMarquardt.hpp"

#include "settings.h"
#include "seekablevideocapture.h"

#include <mutex>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

#include "visualizer.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include "vsp.h"
#include "json_tools.hpp"

#include <memory>
//#include "frequency_domain_optimization.hpp"
using namespace std;

struct videoBufferAndWriter{
    std::mutex mtx;
    volatile bool isWriting;
    std::deque<cv::Mat> images;
    //    std::string videoPass;
    cv::VideoWriter vw;
};

struct strMultiThreadVideoCapture{
    std::mutex mtx;
    std::deque<cv::Mat> images;
    int32_t maxLength;
    cv::VideoCapture vc;
};

videoBufferAndWriter buffer;
strMultiThreadVideoCapture mtvc;


void videoWriterProcess(){
    cv::Mat _buf;
    while(1){//繰り返し書き込み
        {
            std::lock_guard<std::mutex> lock(buffer.mtx);
            //bufferにデータがあるか確認
            if(buffer.images.size()!=0){
                //先頭をコピー
                _buf = buffer.images.front().clone();
                //先頭を削除
                buffer.images.pop_front();
            }else if(!buffer.isWriting){
                return;
            }
        }
        //mutexがunlockされたあとにゆっくりvideoWriterに書き込み
        if(!_buf.empty()){
            buffer.vw << _buf;
            _buf = cv::Mat();
        }else{
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}



void show_correlation(std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> &angularVelocityIn60Hz, std::vector<cv::Vec3d> estimatedAngularVelocity, double Tvideo, double Tav, double frame){

    auto angularVelocity = [&angularVelocityIn60Hz, Tvideo, Tav](double frame_){
        double dframe = frame_ * Tvideo / Tav;
        int i = floor(dframe);
        double decimalPart = dframe - (double)i;
        return angularVelocityIn60Hz[i]*(1.0-decimalPart)+angularVelocityIn60Hz[i+1]*decimalPart;
    };

    for(double d=-0.5;d<=0.5;d+=0.01)
    {

        double sum = 0.0;
        for(int32_t i=0; i<estimatedAngularVelocity.size();i++){
            sum +=   abs(angularVelocity(i+frame+d)[0]-estimatedAngularVelocity[i][0])
                    + abs(angularVelocity(i+frame+d)[1]-estimatedAngularVelocity[i][1])
                    + abs(angularVelocity(i+frame+d)[2]-estimatedAngularVelocity[i][2]);
        }
        cout << "position"<<frame+d<<" minimum correlationCoefficients:" << sum << endl;
    }
}

std::string getVideoSize(const char *videoName){
    std::shared_ptr<cv::VideoCapture> Capture = std::make_shared<cv::VideoCapture>(videoName);//動画をオープン
    assert(Capture->isOpened());
    std::string videoSize = std::to_string((int)Capture->get(cv::CAP_PROP_FRAME_WIDTH)) + std::string("x") + std::to_string((int)Capture->get(cv::CAP_PROP_FRAME_HEIGHT));
    return videoSize;
}

int getVideoLength(const char *videoName){
    std::shared_ptr<cv::VideoCapture> Capture = std::make_shared<cv::VideoCapture>(videoName);//動画をオープン
    assert(Capture->isOpened());
    return (int)Capture->get(cv::CAP_PROP_FRAME_COUNT);
}

int main(int argc, char** argv){
    bool debug_signal_processing = false;
    bool enable_quality_check_tool = false;
    //テクスチャ座標の準備
    int32_t division_x = 9; //画面の横の分割数
    int32_t division_y = 9; //画面の縦の分割数
    cv::Size textureSize = cv::Size(2048,2048);

    //画面の補正量
    float zoomRatio = 1.f;
//    double rollingShutterDuration = 0; //rolling shutter duration [frame]
//    int32_t lowPassFilterStrength = 3;
    //引数の確認
    char *videoPass = NULL;
    char *jsonPass = NULL;
    char *cameraName = NULL;
    char *lensName = NULL;
    //    char *outputPass = NULL;
    bool outputStabilizedVideo = false;
    int opt;
    while((opt = getopt(argc, argv, "j:i:c:l:o::d::z:f:q::")) != -1){
        string value1 ;//= optarg;
        switch (opt) {
        case 'j':       //input json file from virtual gimbal
            jsonPass = optarg;
            break;
        case 'i':       //input video file pass
            videoPass = optarg;
            break;
        case 'c':       //camera name
            cameraName = optarg;
            break;
        case 'l':
            lensName = optarg;
            break;
        case 'o':       //output
            outputStabilizedVideo = true;
            break;
        case 'd':
            debug_signal_processing = true;
            break;
        case 'z':       //zoom ratio, dafault 1.0
            value1 = optarg;
            zoomRatio = std::stof(value1);
            break;
        case 'q':
            enable_quality_check_tool = true;
            break;
        default :
            printf(     "virtualGimbal\r\n"
                        "Post processing video stabilizer\r\n\r\n"
                        "usage: virtualGimbal [-j Angular velocity json file name] [-i video filename] [-c camera name] [-l lens name] [-d debug] [-z zoom ratio] [-q run quality check tool] [[output option] -o] [options]\r\n"
                        );
            return 1;
        }
    }

    std::string videoSize = getVideoSize(videoPass);

    // Read camera calibration information from a json file, and generate cameraInfo class instance.

    shared_ptr<CameraInformation> cameraInfo(new CameraInformationJsonParser(cameraName,lensName,videoSize.c_str()));
    std::cout << "camera_name_" << cameraInfo->camera_name_ << std::endl;
    std::cout << "lens_name_" << cameraInfo->lens_name_ << std::endl;
    std::cout << "width_" << cameraInfo->width_ << std::endl;
    std::cout << "height_" << cameraInfo->height_ << std::endl;
    std::cout << "fx_" << cameraInfo->fx_ << std::endl;
    std::cout << "fy_" << cameraInfo->fy_ << std::endl;

    // std::dynamic_pointer_cast<CameraInformationJsonParser>(cameraInfo)->writeCameraInformationJson("camera_descriptions/cameras.json");

    std::vector<cv::Vec3d> opticShift;
    //動画からオプティカルフローを計算する
    int syncLength = getVideoLength(videoPass) < SYNC_LENGTH ? getVideoLength(videoPass) : SYNC_LENGTH;
    auto t1 = std::chrono::system_clock::now() ;
    Eigen::MatrixXd optical_flow;
    if(jsonExists(std::string(videoPass))){
        readOpticalFlowFromJson(optical_flow,std::string(videoPass));
//        std::cout << optical_flow2 << std::endl;
        opticShift.resize(optical_flow.rows());
        for(int row=0;row<optical_flow.rows();++row){
            for(int col=0;col<3;++col){
                opticShift[row][col] = optical_flow(row,col);
            }
        }
    }else{
        opticShift = CalcShiftFromVideo(videoPass,syncLength);//ビデオからオプティカルフローを用いてシフト量を算出
//        Eigen::MatrixXd optical_flow;
        optical_flow.resize(opticShift.size(),3);
//        memcpy(optical_flow.data(),opticShift.data(),sizeof(double)*3*opticShift.size());
        for(int row=0;row<optical_flow.rows();++row){
            for(int col=0;col<3;++col){
                optical_flow(row,col) = opticShift[row][col];
            }
        }
        writeOpticalFrowToJson(optical_flow,std::string(videoPass));
    }
    auto t2 = std::chrono::system_clock::now() ;

//    std::cout << optical_flow << std::endl;

//    return 0;

    // 処理の経過時間
    auto elapsed = t2 - t1 ;
    std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << " ms\n";
    //show(opticShift,"opticShift");
    cv::VideoCapture *Capture = new cv::VideoCapture(videoPass);//動画をオープン
    assert(Capture->isOpened());
    cv::Size imageSize = cv::Size(Capture->get(cv::CAP_PROP_FRAME_WIDTH),Capture->get(cv::CAP_PROP_FRAME_HEIGHT));//解像度を読む
    std::cout << "Static size:" << textureSize << std::endl;

    //テキスチャーのサイズをここで計算する
    textureSize.width = pow(2.0,ceil(log(imageSize.width)/log(2.0)));
    textureSize.height = pow(2.0,ceil(log(imageSize.height)/log(2.0)));
    std::cout << "Adaptive size:" << textureSize << std::endl;


    double Tvideo = 1.0/Capture->get(cv::CAP_PROP_FPS);
    std::cout << "resolution" << imageSize << std::endl;
    std::cout << "samplingPeriod" << Tvideo << std::endl;



    //動画書き出しのマルチスレッド処理の準備
    buffer.isWriting = true;
    if(outputStabilizedVideo){
        std::string outputPass= videoPass;
        outputPass = outputPass + "_deblured.avi";
        buffer.vw = cv::VideoWriter(outputPass,cv::VideoWriter::fourcc('F', 'M', 'P', '4'),1/Tvideo,cv::Size(imageSize.width,imageSize.height),true);
        if(!buffer.vw.isOpened()){
            printf("Error:Can't Open Video Writer.");
            return -1;
        }
    }
    std::thread th1;
    if(outputStabilizedVideo){
        th1 = std::thread(videoWriterProcess);//スレッド起動
    }
    //逆歪パラメータの計算
    cv::Mat matInvDistort;
    calcInverseDistortCoeff(*cameraInfo);

    //逆歪パラメータ表示
    if(PRINT_INV_DISTORT_COEFF){
        cout << "distCoeff:" << cameraInfo->k1_ << "," <<cameraInfo->k2_ << "," <<cameraInfo->p1_ << "," <<cameraInfo->p2_ << "," << endl;
        cout << "invert distCoeff:" << cameraInfo->inverse_k1_ << "," <<cameraInfo->inverse_k2_ << "," <<cameraInfo->inverse_p1_ << "," <<cameraInfo->inverse_p2_ << "," << endl;
    }


    //角速度データを読み込み

    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> angular_velocity_from_csv;
    if(jsonPass){
        readAngularVelocityFromJson(angular_velocity_from_csv,jsonPass);
    }else{
        std::cout << "csv or json file are required." << std::endl;
        return 0;
    }

    vsp::angularVelocityCoordinateTransformer(angular_velocity_from_csv,cameraInfo->sd_card_rotation_);

    for(int i=0;i<10;i++){
        printf("%f %f %f\n",angular_velocity_from_csv[i][0],angular_velocity_from_csv[i][1],angular_velocity_from_csv[i][2]);
    }
    
    std::cout << "length:" << angular_velocity_from_csv.size() << std::endl << std::flush;

    //ジャイロのDCオフセット（いわゆる温度ドリフトと等価）を計算。単純にフレームの平均値を計算
    if(SUBTRACT_OFFSET){
        Eigen::Vector3d dc(0,0,0);
        for(auto el:angular_velocity_from_csv){
            dc[0] += el[0];
            dc[1] += el[1];
            dc[2] += el[2];
        }
        dc[0]/=angular_velocity_from_csv.size();
        dc[1]/=angular_velocity_from_csv.size();
        dc[2]/=angular_velocity_from_csv.size();
        for(auto &el:angular_velocity_from_csv){
            el[0] -= dc[0];
            el[1] -= dc[1];
            el[2] -= dc[2];
        }
    }

    double Tav = 1./readSamplingRateFromJson(jsonPass);//Sampling period of angular velocity

    //動画のサンプリング周期に合わせて、角速度を得られるようにする関数を定義
    //線形補間
    auto angularVelocity = [&angular_velocity_from_csv, Tvideo, Tav](uint32_t frame){
        double dframe = frame * Tvideo / Tav;
        int i = floor(dframe);
        double decimalPart = dframe - (double)i;
        return angular_velocity_from_csv[i]*(1.0-decimalPart)+angular_velocity_from_csv[i+1]*decimalPart;
    };


    //動画のオプティカルフローと内部パラメータと解像度から角速度推定値を計算
    vector<cv::Vec3d> estimatedAngularVelocity;
    for(auto el:opticShift){
        estimatedAngularVelocity.push_back(cv::Vec3d(-atan(el[1]/cameraInfo->fy_),atan(el[0]/cameraInfo->fx_),el[2])/Tvideo*-1);
    }

    t1 = std::chrono::system_clock::now() ;

    int32_t angular_velocity_length_in_video = ceil(angular_velocity_from_csv.size() * Tav / Tvideo);


    int32_t lengthDiff = angular_velocity_length_in_video - estimatedAngularVelocity.size();
    cout << "lengthDiff:" << lengthDiff << endl;

    t2 = std::chrono::system_clock::now() ;

    //ここでEigenのMatrixでできるだけ計算するルーチンを追加してみる
    Eigen::MatrixXd angular_velocity_matrix             = Eigen::MatrixXd::Zero(ceil(angular_velocity_from_csv.size()*Tav /Tvideo),3);
    for(int32_t i=0;i<angular_velocity_length_in_video;++i){
        angular_velocity_matrix.row(i) = angularVelocity(i).transpose();
    }
    Eigen::MatrixXd estimated_angular_velocity_matrix   = Eigen::Map<Matrix<double, Eigen::Dynamic, Eigen::Dynamic, RowMajor>>((double*)(estimatedAngularVelocity.data()),estimatedAngularVelocity.size(),3);
    vector<double> correlation_coefficients(lengthDiff);
//    if((!syncronizedQuarternionExist(videoPass)) || debug_signal_processing){
    std::cout << "\nCaluclating correlation cofficient..." << std::endl;
        for(int32_t offset=0;offset<lengthDiff;++offset){
            correlation_coefficients[offset] = (angular_velocity_matrix.block(offset,0,estimated_angular_velocity_matrix.rows(),estimated_angular_velocity_matrix.cols())
                                                -estimated_angular_velocity_matrix).array().abs().sum();
            if(offset%100==0){
                printf("\r%d / %d",offset,lengthDiff);
                std::cout << std::flush;
            }
        }
        printf("\r%d / %d\n",lengthDiff,lengthDiff);
//    }
    int32_t min_position = std::distance(correlation_coefficients.begin(),min_element(correlation_coefficients.begin(),correlation_coefficients.end()));


    auto t3 = std::chrono::system_clock::now() ;
    // 処理の経過時間
    elapsed = t3 - t2 ;
    std::cout << "Elapsed time@search minimum in new method: " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << " ms" << std::endl;



    //最小値サブピクセル推定
    double subframeOffset;
    if(min_position == 0){	//位置が最初のフレームで一致している場合
        subframeOffset = 0.0;
    }else if(min_position == (lengthDiff-1)){//末尾
        subframeOffset = (double)(lengthDiff -1);
    }else{					//その他
        subframeOffset = -(correlation_coefficients[min_position+1]-correlation_coefficients[min_position-1])/(2*correlation_coefficients[min_position-1]-4*correlation_coefficients[min_position]+2*correlation_coefficients[min_position+1]);
    }
    cout << "min_position" << min_position << endl;
//    cout << "minPosition" << minPosition << endl;
    cout << "subframe minposition :" << min_position+subframeOffset << endl;

    if(debug_signal_processing) show_correlation(angular_velocity_from_csv,estimatedAngularVelocity, Tvideo,Tav,min_position+subframeOffset);


    //EigenによるDFT LPFのテスト
    if(debug_signal_processing){

        vsp v2(/*angleQuaternion_vsp2,*/
               division_x,
               division_y,
               *cameraInfo,
               (double)zoomRatio,
               angular_velocity_from_csv,
               Tvideo,
               Tav,
               min_position + subframeOffset,
               (int32_t)(Capture->get(cv::CAP_PROP_FRAME_COUNT)),
               199);
        v2.setParam(Capture->get(cv::CAP_PROP_FPS),1.0);
        std::vector<string> legends_quaternion = {"x","y","z","w"};
        vgp::plot(v2.toQuaternion(),"Raw Quaternion",legends_quaternion);
        vgp::plot(v2.filteredQuaternion(100),"Filtered Quaternion with constant filter coefficient : 100",legends_quaternion);

        std::vector<string> legends2 = {"x"};

        v2.setMaximumGradient(0.5);
        Eigen::VectorXd filter_coefficients = v2.calculateFilterCoefficientsWithoutBlackSpaces(2,499);
        vgp::plot(filter_coefficients,std::string("Filter coefficients (Lower is Strong stabilization), scale:")+std::to_string(zoomRatio),legends2);
        vgp::plot(v2.filteredQuaternion(filter_coefficients),std::string("Filtered Quaternion with variable filter coefficient, scale:")+std::to_string(zoomRatio),legends_quaternion);


        return 0;
    }

    shared_ptr<vsp> v2;

        v2.reset(new vsp(division_x,
                         division_y,
                         *cameraInfo,
                         (double)zoomRatio,
                         angular_velocity_from_csv,
                         Tvideo,
                         Tav,
                         min_position + subframeOffset,
                         (int32_t)(Capture->get(cv::CAP_PROP_FRAME_COUNT)),
                         199));

        // Stabilize
    Eigen::VectorXd filter_coefficients = v2->calculateFilterCoefficientsWithoutBlackSpaces(2,499);
    v2->filteredQuaternion(filter_coefficients);

    if(enable_quality_check_tool){
        printf("index,ex,ey,ez,x,y,z\n");
        for(int row=0;row<estimated_angular_velocity_matrix.rows();++row){
            // std::cout << row << "," << estimated_angular_velocity_matrix.row(row) << "," << v2->angularVelocitySync(row).transpose() <<std::endl;
            printf("%d,%f,%f,%f,%f,%f,%f\n",row,estimated_angular_velocity_matrix(row,0),estimated_angular_velocity_matrix(row,1),estimated_angular_velocity_matrix(row,2),
            v2->angularVelocitySync(row)[0],v2->angularVelocitySync(row)[1],v2->angularVelocitySync(row)[2]);
        }

        Eigen::MatrixXd angular_velocity(estimated_angular_velocity_matrix.rows(),6);
        for(int row=0;row<estimated_angular_velocity_matrix.rows();++row){
            angular_velocity(row,0) = estimated_angular_velocity_matrix(row,0);
            angular_velocity(row,1) = estimated_angular_velocity_matrix(row,1);
            angular_velocity(row,2) = estimated_angular_velocity_matrix(row,2);
            angular_velocity(row,3) = v2->angularVelocitySync(row)[0];
            angular_velocity(row,4) = v2->angularVelocitySync(row)[1];
            angular_velocity(row,5) = v2->angularVelocitySync(row)[2];
        }

        std::vector<string> legends_angular_velocity = {"ex","ey","ez","x","y","z"};
        vgp::plot(angular_velocity,"Estimate vs gyro",legends_angular_velocity);
    }

    v2->init_opengl(textureSize);

    //一度動画を閉じて、seek可能版に置き換える
    int32_t e=Capture->get(cv::CAP_PROP_FRAME_COUNT);
    delete Capture;
//    seekableVideoCapture sCapture(videoPass,PREFETCH_LENGTH);


    cv::namedWindow("Preview",cv::WINDOW_NORMAL);
    cv::setWindowProperty("Preview",cv::WND_PROP_FULLSCREEN,cv::WINDOW_FULLSCREEN);
//    while(v2->ok()){
        Capture = new cv::VideoCapture(videoPass);//動画をオープン
        for(int32_t i=0;i<e;++i){
            cv::Mat simg;
            if(0 != v2->spin_once(i,*Capture,simg)){
                break;
            }

            if(outputStabilizedVideo){
                std::lock_guard<std::mutex> lock(buffer.mtx);
                buffer.images.push_back(cv::Mat());
                buffer.images.back() = simg.clone();
            }


            //Show fps
            auto t4 = std::chrono::system_clock::now();
            static auto t3 = t4;
            // 処理の経過時間
            double elapsedmicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() ;
            static double fps = 0.0;
            if(elapsedmicroseconds != 0.0){
                fps = 0.03*(1e6/elapsedmicroseconds) +  0.97*fps;
            }
            t3 = t4;
            printf("fps:%4.2f\r",fps);
            fflush(stdout);


        }

        if(Capture != NULL){
            delete Capture;
            Capture = NULL;
        }

//    }

    if(Capture != NULL){
        delete Capture;
        Capture = NULL;
    }
    v2->stop_opengl();

    //動画書き出しのマルチスレッド処理の終了処理
    //書き込みが終わっているか確認
    if(outputStabilizedVideo){
        cout << "Waiting for videoWriter." << endl;
        buffer.isWriting = false;
        th1.join();
    }

#if MULTITHREAD_CAPTURE
    {
        cout << "Wainting for videoCapture." << endl;
        th2.join();
    }
#endif
    cv::destroyAllWindows();

    //音声を付加
    //inputVideoPassにスペースがあったらエスケープシーケンスを追加
    std::string::size_type pos = 0;
    std::string sInputVideoPass = videoPass;
    while(pos=sInputVideoPass.find(" ",pos), pos!=std::string::npos){
        sInputVideoPass.insert(pos,"\\");
        pos+=2;
    }

    //#define OUTPUTVIDEO
    //#ifdef OUTPUTVIDEO
    if(outputStabilizedVideo){
        std::cout << "Separate audio" << std::endl;
        std::string command = "ffmpeg -i " + sInputVideoPass +  " -vn -acodec pcm_s16le output-audio.wav";
        system(command.c_str());
        std::cout << "Combine audio" << std::endl;
        command = "ffmpeg -i " + sInputVideoPass + "_deblured.avi -i output-audio.wav -codec copy " + sInputVideoPass + "_deblured_audio.avi";
        system(command.c_str());

        system("rm output-audio.wav");
        command = "rm " + sInputVideoPass + "_deblured.avi";
        system(command.c_str());
    }
    //#endif
    return 0;
}

