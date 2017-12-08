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
GLFWwindow* window;

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
//void calcDistortCoeff(const cv::Mat &matIntrinsic, const cv::Mat &matDistort, const cv::Size &imageSize, cv::Mat &matInvDistort){
//    //逆歪パラメータを求める
//    std::vector<double> refPointsX;
//    std::vector<double> refPointsY;
//    Matrix3d intrinsic;
//    intrinsic << 	matIntrinsic.at<double>(0,0),matIntrinsic.at<double>(0,1),matIntrinsic.at<double>(0,2),
//            matIntrinsic.at<double>(1,0),matIntrinsic.at<double>(1,1),matIntrinsic.at<double>(1,2),
//            matIntrinsic.at<double>(2,0),matIntrinsic.at<double>(2,1),matIntrinsic.at<double>(2,2);

//    std::cout << intrinsic << std::endl;
//    VectorXd distortionCoeff(4);
//    distortionCoeff << matDistort.at<double>(0,0),matDistort.at<double>(0,1),matDistort.at<double>(0,2),matDistort.at<double>(0,3);
//    std::cout << distortionCoeff << std::endl;
//    int step = 20;
//    for(int v=0;v<=imageSize.height;v+=step){
//        for(int u=0;u<=imageSize.width;u+=step){
//            refPointsX.push_back((double)u);
//            refPointsY.push_back((double)v);
//        }
//    }
//    //歪補正
//    std::vector<double> undistortedPointsX;
//    std::vector<double> undistortedPointsY;
//    double fx = intrinsic(0, 0);
//    double fy = intrinsic(1, 1);
//    double cx = intrinsic(0, 2);
//    double cy = intrinsic(1, 2);
//    double k1 = distortionCoeff(0);
//    double k2 = distortionCoeff(1);
//    double p1 = distortionCoeff(2);
//    double p2 = distortionCoeff(3);
//    for(int i=0,e=refPointsX.size();i<e;i++){
//        double u = refPointsX[i];
//        double v = refPointsY[i];

//        double x1 = (u - cx)/fx;
//        double y1 = (v - cy)/fy;

//        double r = sqrt(x1*x1+y1*y1);

//        double x2 = x1*(1.0+k1*r*r+k2*r*r*r*r)+2.0*p1*x1*y1+p2*(r*r+2.0*x1*x1);
//        double y2 = y1*(1.0+k1*r*r+k2*r*r*r*r)+p1*(r*r+2.0*y1*y1)+2.0*p2*x1*y1;
//        double mapx = x2*fx+cx;
//        double mapy = y2*fy+cy;
//        undistortedPointsX.push_back(mapx);
//        undistortedPointsY.push_back(mapy);
//    }
//    //最適化
//    printf("Before:\t%f,%f,%f,%f\r\n",distortionCoeff[0],distortionCoeff[1],distortionCoeff[2],distortionCoeff[3]);
//    calc_invert_distortion_coeff functor2(distortionCoeff.size(),refPointsX.size(), undistortedPointsX, undistortedPointsY,
//                                          refPointsX, refPointsY, intrinsic);

//    NumericalDiff<calc_invert_distortion_coeff> numDiff2(functor2);
//    LevenbergMarquardt<NumericalDiff<calc_invert_distortion_coeff> > lm2(numDiff2);
//    int info = lm2.minimize(distortionCoeff);
//    printf("After:\t%f,%f,%f,%f\r\n",distortionCoeff[0],distortionCoeff[1],distortionCoeff[2],distortionCoeff[3]);

//    matInvDistort = (cv::Mat_<double>(1, 4) << distortionCoeff[0],distortionCoeff[1],distortionCoeff[2],distortionCoeff[3]);
//}

/**
 * @param 回転を表すクォータニオンをシングルローテーションをあらわすベクトルへ変換
 **/
template <typename T_num> cv::Vec3d Quaternion2Vector(quaternion<T_num> q){
    double denom = sqrt(1-q.R_component_1()*q.R_component_1());
    if(denom==0.0){//まったく回転しない時は０割になるので、場合分けする
        return cv::Vec3d(0,0,0);//return zero vector
    }
    return cv::Vec3d(q.R_component_2(),q.R_component_3(),q.R_component_4())*2.0*atan2(denom,q.R_component_1())/denom;
}

/**
 * @param シングルローテーションを表すベクトルを回転を表すクォータニオンへ変換
 **/
template <typename T_num> quaternion<T_num> Vector2Quaternion(cv::Vec3d w){
    double theta = sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]);//回転角度を計算、normと等しい
    //0割を回避するためにマクローリン展開
    if(theta > EPS){
        auto n = w * (1.0/theta);//単位ベクトルに変換
        double sin_theta_2 = sin(theta*0.5);
        return quaternion<double>(cos(theta*0.5),n[0]*sin_theta_2,n[1]*sin_theta_2,n[2]*sin_theta_2);
    }else{
        return quaternion<double>(1.0,0.5*w[0],0.5*w[1],0.5*w[2]);
    }
}

/**
 * @param 回転を表すクォータニオンをシングルローテーションを表すベクトルへ変換。前回算出したベクトルを引数として受け取ることで、アンラッピングする。
 * */
template <typename T_num> cv::Vec3d Quaternion2Vector(quaternion<T_num> q, cv::Vec3d prev){
    double denom = sqrt(1-q.R_component_1()*q.R_component_1());
    if(denom==0.0){//まったく回転しない時は０割になるので、場合分けする
        return cv::Vec3d(0,0,0);//return zero vector
    }
    double theta_2 = atan2(denom,q.R_component_1());
    double prev_theta_2 = cv::norm(prev)/2;
    double diff = theta_2 - prev_theta_2;
    theta_2 -= 2.0*M_PI*(double)(static_cast<int>(diff/(2.0*M_PI)));//マイナスの符号に注意
    //~ printf("Theta_2:%4.3f sc:%d\n",theta_2,static_cast<int>(diff/(2.0*M_PI)));
    if(static_cast<int>(diff/(2.0*M_PI))!=0){
        printf("\n###########Unwrapping %d\n",static_cast<int>(diff/(2.0*M_PI)));
    }

    return cv::Vec3d(q.R_component_2(),q.R_component_3(),q.R_component_4())*2.0*theta_2/denom;
}


///**
// * @param 回転を表すクォータニオンから回転を表す行列を生成
// **/
//template <typename T_num> void Quaternion2Matrix(quaternion<T_num> q, cv::Mat &det){
//    det = (cv::Mat_<T_num>(3,3) <<
//           q.R_component_1()*q.R_component_1()+q.R_component_2()*q.R_component_2()-q.R_component_3()*q.R_component_3()-q.R_component_4()*q.R_component_4(), 2*(q.R_component_2()*q.R_component_3()-q.R_component_1()*q.R_component_4()),                  2*(q.R_component_2()*q.R_component_4()+q.R_component_1()*q.R_component_3()),
//           2*(q.R_component_2()*q.R_component_3()+q.R_component_1()*q.R_component_4()),                 q.R_component_1()*q.R_component_1()-q.R_component_2()*q.R_component_2()+q.R_component_3()*q.R_component_3()-q.R_component_4()*q.R_component_4(), 2*(q.R_component_3()*q.R_component_4()-q.R_component_1()*q.R_component_2()),
//           2*(q.R_component_2()*q.R_component_4()-q.R_component_1()*q.R_component_3()),                 2*(q.R_component_3()*q.R_component_4()+q.R_component_1()*q.R_component_2()),                  q.R_component_1()*q.R_component_1()-q.R_component_2()*q.R_component_2()-q.R_component_3()*q.R_component_3()+q.R_component_4()*q.R_component_4()
//           );
//}

///**
// * @brief 球面線形補間関数
// * @param [in]	Qfrom	四元数1
// * @param [in]	Qto		四元数2
// * @param [in]	t		比率(0<=t<=1)
// **/
//template <typename _Tp> quaternion<_Tp> Slerp(quaternion<_Tp> Qfrom, quaternion<_Tp> Qto, _Tp t){
//    double cosom = Qfrom.R_component_1()*Qto.R_component_1()+Qfrom.R_component_2()*Qto.R_component_2()+Qfrom.R_component_3()*Qto.R_component_3()+Qfrom.R_component_4()*Qto.R_component_4();
//    double sinom, omega, scale0, scale1;

//    if(Qto == Qfrom){	//QfromとQtoが完全に一致->補完の必要なし
//        return Qfrom;
//    }

//    //符号を直す
//    if(cosom < 0.0){
//        cosom = -cosom;
//        Qto = -Qto;
//    }
//    if((1.0-cosom)>1e-4){
//        omega = acos(cosom);
//        sinom = sin(omega);
//        scale0 = sin((1.0 - t) * omega) / sinom;
//        scale1 = sin(t * omega) / sinom;
//    }else{
//        scale0 = 1.0 -t;
//        scale1 = t;
//    }

//    return scale0 * Qfrom + scale1 * Qto;

//}


///** @brief 補正前の画像座標から、補正後のポリゴンの頂点の位置を表す座標の組を作成
// * @param [in]	Qa	ジャイロの角速度から計算したカメラの方向を表す回転クウォータニオン時系列データ、参照渡し
// * @param [in]	Qf	LPFを掛けて平滑化した回転クウォータニオンの時系列データ、参照渡し
// * @param [in]	m	画面の縦の分割数[ ]
// * @param [in]	n	画面の横の分割数[ ]
// * @param [in]	ti	角度時系列データのサンプル時間[sec]
// * @param [in]	ts	ローリングシャッターの全行取得時間[sec]
// * @param [in]	T	ジャイロ角度のサンプリング周期
// * @param [in]	IK	"逆"歪係数(k1,k2,p1,p2)
// * @param [in]	matIntrinsic	カメラ行列(fx,fy,cx,cy) [pixel]
// * @param [in]	imageSize	フレーム画像のサイズ[pixel]
// * @param [in]  adjustmentQuaternion 画面方向を微調整するクォータニオン[rad]
// * @param [out]	vecPorigonn_uv	OpenGLのポリゴン座標(u',v')座標(-1~1)の組、歪補正後の画面を分割した時の一つ一つのポリゴンの頂点の組
// * @param [in]	zoom	倍率[]。拡大縮小しないなら1を指定すること。省略可
// **/
//template <typename _Tp, typename _Tx> void getDistortUnrollingMap(
//        //    std::vector<quaternion<_Tp>> &Qa,
//        //    std::vector<quaternion<_Tp>> &Qf,
//        quaternion<_Tp> &prevAngleQuaternion,
//        quaternion<_Tp> &currAngleQuaternion,
//        quaternion<_Tp> &nextAngleQuaternion,
//        uint32_t division_x,
//        uint32_t division_y,
//        double TRollingShutter,
//        //    double ti,
//        //    double ts,
//        //    double T,
//        cv::Mat &IK,
//        cv::Mat &matIntrinsic,
//        cv::Size imageSize,
//        quaternion<_Tp> adjustmentQuaternion,
//        std::vector<_Tx> &vecPorigonn_uv,
//        double zoom
//        ){

//    //Matの型をdoubleに強制。
//    assert(IK.type() == CV_64F);
//    assert(matIntrinsic.type() == CV_64F);

//    //手順
//    //1.補正前画像を分割した時の分割点の座標(pixel)を計算
//    //2.1の座標を入力として、各行毎のW(t1,t2)を計算
//    //3.補正後の画像上のポリゴン座標(pixel)を計算、歪み補正も含める

//    double fx = matIntrinsic.at<double>(0, 0);
//    double fy = matIntrinsic.at<double>(1, 1);
//    double cx = matIntrinsic.at<double>(0, 2);
//    double cy = matIntrinsic.at<double>(1, 2);
//    double k1 = IK.at<double>(0,0);
//    double k2 = IK.at<double>(0,1);
//    double p1 = IK.at<double>(0,2);
//    double p2 = IK.at<double>(0,3);

//    cv::Mat map(division_y+1,division_x+1,CV_64FC2);


//    for(int j=0;j<=division_y;++j){
//        //W(t1,t2)を計算
//        cv::Mat R;
//        //1
//        double v = (double)j/division_y*imageSize.height;

//        double exposureTimingInEachRow = TRollingShutter*v/imageSize.height;	//ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]

//        quaternion<_Tp> slerpedAngleQuaternion;
//        if(exposureTimingInEachRow >= 0){
//            slerpedAngleQuaternion = Slerp(currAngleQuaternion,nextAngleQuaternion,exposureTimingInEachRow);
//        }else{
//            slerpedAngleQuaternion = Slerp(prevAngleQuaternion,currAngleQuaternion,1+exposureTimingInEachRow);
//        }

//        //        unsigned int fi = (int)floor(exposureTimingInEachRow/T);	//ローリングシャッター補正を含むフレーム数の整数部[ ]
//        //        double ff = exposureTimingInEachRow/T - (double)fi;			//ローリングシャッター補正を含むフレーム数の浮動小数点数部[ ]
//        //        auto SQa = Slerp(Qa[fi],Qa[fi+1],ff);	//オリジナルの角度クウォータニオンに関して球面線形補間

//        //        unsigned int gi = (int)floor(ti/T);		//フレーム数の整数部[ ]
//        //        double gf = ti/T - (double)gi;			//フレーム数の浮動小数点数部[ ]
//        //        auto SQf = Slerp(Qf[gi],Qf[gi+1],gf);	//フィルタ済みの角度クウォータニオンに関して球面線形補間

//        //        Quaternion2Matrix(conj(SQf)*SQa,R);		//ローリングシャッター補正を含む回転行列を計算

//        slerpedAngleQuaternion = adjustmentQuaternion * slerpedAngleQuaternion;

//        Quaternion2Matrix(slerpedAngleQuaternion,R);

//        for(int i=0;i<=division_x;++i){
//            double u = (double)i/division_x*imageSize.width;
//            //後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
//            cv::Mat p = (cv::Mat_<double>(3,1) << (u- cx)/fx, (v - cy)/fy, 1.0);	//1のポリゴン座標に、K^-1を掛けた結果の３x１行列
//            //2
//            cv::Mat XYW = R * p;//inv()なし

//            double x1 = XYW.at<double>(0, 0)/XYW.at<double>(2, 0);
//            double y1 = XYW.at<double>(1, 0)/XYW.at<double>(2, 0);

//            double r = sqrt(x1*x1+y1*y1);

//            double x2 = x1*(1.0+k1*r*r+k2*r*r*r*r)+2.0*p1*x1*y1+p2*(r*r+2.0*x1*x1);
//            double y2 = y1*(1.0+k1*r*r+k2*r*r*r*r)+p1*(r*r+2.0*y1*y1)+2.0*p2*x1*y1;
//            //変な折り返しを防止
//            if((pow(x2-x1,2)>1.0)||(pow(y2-y1,2)>1.0)){
//                //                printf("折り返し防止\r\n");
//                x2 = x1;
//                y2 = y1;
//            }
//            //            double mapx = x2*fx*zoom+cx;
//            //            double mapy = y2*fy*zoom+cy;
//            //~ return ;
//            //結果をmapに保存
//            //            map.at<cv::Vec2d>(j,i)[0] = mapx/textureSize.width;
//            //            map.at<cv::Vec2d>(j,i)[1] = mapy/textureSize.height;
//            //            map.at<cv::Vec2d>(j,i)[0] = (mapx-cx)/imageSize.width*2.0;
//            //            map.at<cv::Vec2d>(j,i)[1] = (mapy-cy)/imageSize.height*2.0;
//            map.at<cv::Vec2d>(j,i)[0] = x2*fx*zoom/imageSize.width*2.0;
//            map.at<cv::Vec2d>(j,i)[1] = y2*fy*zoom/imageSize.height*2.0;
//            //~ printf("i:%d,j:%d,mapx:%4.3f,mapy:%4.3f\n",i,j,mapx,mapy);
//        }
//    }

//    //    cout << map << "\r\n" << endl;

//    //3.ポリゴン座標をOpenGLの関数に渡すために順番を書き換える
//    vecPorigonn_uv.clear();
//    for(int j=0;j<division_y;++j){//jは終了の判定が"<"であることに注意
//        for(int i=0;i<division_x;++i){
//            //GL_TRIANGLESでGL側へ送信するポリゴンの頂点座標を準備
//            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j,i)[0]);//x座標
//            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j,i)[1]);//y座標
//            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j,i+1)[0]);//x座標
//            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j,i+1)[1]);//y座標
//            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j+1,i)[0]);//x座標
//            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j+1,i)[1]);//y座標

//            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j+1,i)[0]);//x座標
//            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j+1,i)[1]);//y座標
//            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j,i+1)[0]);//x座標
//            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j,i+1)[1]);//y座標
//            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j+1,i+1)[0]);//x座標
//            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j+1,i+1)[1]);//y座標


//        }
//    }




//}

/**
 * @brief 回転を表すクォータニオンを生成する関数
 **/
template <typename T_num,typename T_vec> quaternion<T_num> RotationQuaternion(T_num theta, T_vec n){
    //nを規格化する
    double tmp = 1.0/sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);
    n = n * tmp;
    return quaternion<T_num>(cos(theta/2),n[0]*sin(theta/2),n[1]*sin(theta/2),n[2]*sin(theta/2));
}

/**
 * @brief 微小回転を表す回転ベクトルから四元数を作る関数
 **/

template <typename _Tp, int cn> quaternion<_Tp> RotationQuaternion(cv::Vec<_Tp,cn> w){
    double theta = sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]);	//!<回転角
    if(theta == 0){
        return quaternion<_Tp>(1,0,0,0);
    }
    auto n = w*(1.0/theta);								//!<回転軸を表す単位ベクトル
    return RotationQuaternion(theta, n);
}

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

/**
 * @brief ひたすらビデオを読み込み
 */
void videoCaptureProcess(){
    cv::Mat _buf;
    while(1){
        if(_buf.empty()){
            mtvc.vc >> _buf;//読み出し
        }else{
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            printf("\r\nmtvc.images.size():%ld\r\n",mtvc.images.size());
        }
        std::lock_guard<std::mutex> lock(mtvc.mtx);
        if(mtvc.images.size()<mtvc.maxLength){
            //bufferに画像をコピー
            mtvc.images.push_back(cv::Mat());
            //コピー
            mtvc.images.back() = _buf.clone();
            //もしも空なら動画の読み込みを終了する
            if(mtvc.images.back().empty()){
                return;
            }
            _buf = cv::Mat();
        }

    }
}

int main(int argc, char** argv){



    //テクスチャ座標の準備
    int32_t division_x = 9; //画面の横の分割数
    int32_t division_y = 9; //画面の縦の分割数
    cv::Size textureSize = cv::Size(2048,2048);

    //画面の補正量
    float vAngle = 0.f;
    float hAngle = 0.f;
    float zoomRatio = 1.f;
    double rollingShutterDuration = 0; //rolling shutter duration [frame]
    int32_t lowPassFilterStrength = 3;
    //引数の確認
    char *videoPass = NULL;
    char *csvPass = NULL;
    //    char *outputPass = NULL;
    bool outputStabilizedVideo = false;
    int opt;
    while((opt = getopt(argc, argv, "i:c:o::v:h:z:r:f:")) != -1){
        string value1 ;//= optarg;
        switch (opt) {
        case 'i':       //input video file pass
            videoPass = optarg;
            break;
        case 'c':       //input angular velocity csv file pass
            csvPass = optarg;
            break;
        case 'o':       //output
            outputStabilizedVideo = true;
            break;
        case 'v':       //vertical position adjustment [rad], default 0
            value1 = optarg;
            vAngle = std::stof(value1);
            break;
        case 'h':       //horizontal position adjustment [rad], default 0
            value1 = optarg;
            hAngle = std::stof(value1);
            break;
        case 'z':       //zoom ratio, dafault 1.0
            value1 = optarg;
            zoomRatio = std::stof(value1);
            break;
        case 'r':       //rolling shutter duration [frame]. This should be between -1 and 1.
            value1 = optarg;
            rollingShutterDuration = std::stof(value1);
            if(rollingShutterDuration > 1.0){
                rollingShutterDuration = 1.0;
            }else if(rollingShutterDuration < -1.0){
                rollingShutterDuration = -1.0;
            }
            break;
        case 'f':       //Low pass filter strength of a camera shake reduction.
                        //Larger is strong filter. This parameter must be integer.
                        //Default 3.
            value1 = optarg;
            lowPassFilterStrength = std::stoi(value1);
            if(lowPassFilterStrength < 0){
                cout << "Low pass filter strength must be greater than or equal to 0.\r\n" <<
                     "It is set as 0 automatically." << endl;
                lowPassFilterStrength = 0;
            }else if(lowPassFilterStrength > 11){
                cout << "Low pass filter strength must be less than 12.\r\n" <<
                     "It is set as 11 automatically." << endl;
                lowPassFilterStrength = 1;
            }
            break;
        default :
            printf(     "virtualGimbal\r\n"
                        "Hyper fast video stabilizer\r\n\r\n"
                        "usage: virtualGimbal [-i video] [-f angularVelocity] [[output option] -o] [options]\r\n"
                        );
            return 1;
        }
    }

    //動画からオプティカルフローを計算する
    auto t1 = std::chrono::system_clock::now() ;
    std::vector<cv::Vec3d> opticShift = CalcShiftFromVideo(videoPass,SYNC_LENGTH);//ビデオからオプティカルフローを用いてシフト量を算出
    auto t2 = std::chrono::system_clock::now() ;
    // 処理の経過時間
    auto elapsed = t2 - t1 ;
    std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << " ms\n";

    cv::VideoCapture *Capture = new cv::VideoCapture(videoPass);//動画をオープン
    assert(Capture->isOpened());
    cv::Size imageSize = cv::Size(Capture->get(CV_CAP_PROP_FRAME_WIDTH),Capture->get(CV_CAP_PROP_FRAME_HEIGHT));//解像度を読む
    double Tvideo = 1.0/Capture->get(CV_CAP_PROP_FPS);
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

    //動画書き出しのマルチスレッド処理の準備
    buffer.isWriting = true;
    if(outputStabilizedVideo){
        std::string outputPass= videoPass;
        outputPass = outputPass + "_deblured.avi";
        buffer.vw = cv::VideoWriter(outputPass,CV_FOURCC('F', 'M', 'P', '4'),1/Tvideo,cv::Size(imageSize.width,imageSize.height),true);
        if(!buffer.vw.isOpened()){
            printf("Error:Can't Open Video Writer.");
            return -1;
        }
    }
    std::thread th1;
    if(outputStabilizedVideo){
        th1 = std::thread(videoWriterProcess);//スレッド起動
    }

    //動画読み込みのマルチスレッド処理の準備
#if MULTITHREAD_CAPTURE
    mtvc.maxLength = 500;
    mtvc.vc = cv::VideoCapture(videoPass);//動画をオープン
    std::thread th2(videoCaptureProcess);
#endif

    //歪パラメータの読み込み
    cv::Mat matDist;
    ReadDistortionParams("distortion.txt",matDist);
    std::cout << "Distortion Coeff:\n" << matDist << "\n" << std::endl;

    //逆歪パラメータの計算
    cv::Mat matInvDistort;
    calcDistortCoeff(matIntrinsic,matDist,imageSize,matInvDistort);

    //逆歪パラメータ表示
    if(PRINT_INV_DISTORT_COEFF){
        cout << "distCoeff:" << matDist << endl;
        cout << "invert distCoeff:" << matInvDistort << endl;
    }

    //    cv::Mat img;

    //試しに先に進む
    //    Capture->set(cv::CAP_PROP_POS_FRAMES,500);

    //動画の読み込み
    //    Capture >> img;

    //角速度データを読み込み
    std::vector<cv::Vec3d> angularVelocityIn60Hz;
    ReadCSV(angularVelocityIn60Hz,csvPass);

    //軸の定義方向の入れ替え
    //TODO:将来的に、CSVファイルに順番を揃えて、ゲインも揃えた値を書き込んでおくべき
    //    for(auto &el:angularVelocityIn60Hz){
    //        auto temp = el;
    //        el[0] = temp[1]/16.4*M_PI/180.0;//16.4はジャイロセンサが感度[LSB/(degree/s)]で2000[degrees/second]の時のもの。ジャイロセンサの種類や感度を変更した時は値を変更する;
    //        el[1] = temp[0]/16.4*M_PI/180.0;
    //        el[2] = -temp[2]/16.4*M_PI/180.0;
    //    }

    //ジャイロのDCオフセット（いわゆる温度ドリフトと等価）を計算。単純にフレームの平均値を計算
    if(SUBTRACT_OFFSET){
        cv::Vec3d dc(0,0,0);
        for(auto el:angularVelocityIn60Hz){
            dc[0] += el[0];
            dc[1] += el[1];
            dc[2] += el[2];
        }
        dc[0]/=angularVelocityIn60Hz.size();
        dc[1]/=angularVelocityIn60Hz.size();
        dc[2]/=angularVelocityIn60Hz.size();
        for(auto &el:angularVelocityIn60Hz){
            el[0] -= dc[0];
            el[1] -= dc[1];
            el[2] -= dc[2];
        }
    }

    double Tav = 1/60.0;//Sampling period of angular velocity

    //動画のサンプリング周期に合わせて、角速度を得られるようにする関数を定義
    //線形補間
    auto angularVelocity = [&angularVelocityIn60Hz, Tvideo, Tav](uint32_t frame){
        //        double dframe = frame * Tav / Tvideo;
        double dframe = frame * Tvideo / Tav;
        int i = floor(dframe);
        double decimalPart = dframe - (double)i;
        return angularVelocityIn60Hz[i]*(1.0-decimalPart)+angularVelocityIn60Hz[i+1]*decimalPart;
    };

    //    cout << "angular Velocity" << endl;
    //    for(int i=0;i<1000;i++){
    //        cout << angularVelocity(i) << endl;
    //    }



    //動画のオプティカルフローと内部パラメータと解像度から角速度推定値を計算
    vector<cv::Vec3d> estimatedAngularVelocity;
    //    cout << "estimated AngularVelocity" << endl;
    for(auto el:opticShift){
        estimatedAngularVelocity.push_back(cv::Vec3d(-atan(el[1]/fy),atan(el[0]/fx),el[2])/Tvideo*-1);
        //        cout << estimatedAngularVelocity.back() << endl;
        //        printf("%f,%f,%f\n",estimatedAngularVelocity.back()[0],estimatedAngularVelocity.back()[1],estimatedAngularVelocity.back()[2]);
    }

    //sync test
#if 0
    Tav = Tvideo;
    for(int i=0;i<estimatedAngularVelocity.size();i++){
        angularVelocityIn60Hz[i] = estimatedAngularVelocity[i];
    }

#endif

    t1 = std::chrono::system_clock::now() ;

    //    int32_t lengthDiff = angularVelocityIn60Hz.size() * Tvideo / Tav - estimatedAngularVelocity.size();
    int32_t lengthDiff = angularVelocityIn60Hz.size() * Tav / Tvideo - estimatedAngularVelocity.size();
    cout << "lengthDiff:" << lengthDiff << endl;
    vector<double> correlationCoefficients(lengthDiff);
    double minCC = DBL_MAX;
    for(int32_t offset = 0; offset < lengthDiff; offset++){
        double sum = 0.0;
        for(int32_t i=0; i<estimatedAngularVelocity.size();i++){
            sum +=   abs(angularVelocity(i+offset)[0]-estimatedAngularVelocity[i][0])
                    + abs(angularVelocity(i+offset)[1]-estimatedAngularVelocity[i][1])
                    + abs(angularVelocity(i+offset)[2]-estimatedAngularVelocity[i][2]);
            //            double diff = abs(angularVelocity(i+offset)[0]-estimatedAngularVelocity[i][0])
            //                        + abs(angularVelocity(i+offset)[1]-estimatedAngularVelocity[i][1]);
            //                        + abs(angularVelocity(i+offset)[2]-estimatedAngularVelocity[i][2]);
            //            sum += (diff < 0.05) ? diff : 0.05;
            if(sum > minCC){
                break;
            }
        }
        if(sum < minCC){
            minCC = sum;
        }
        correlationCoefficients[offset] = sum;
    }



    //    cout << "correlationCoefficients" << endl;
    //    for(auto el:correlationCoefficients) cout << el << endl;

    //最小となる要素を取得
    int32_t minPosition = std::distance(correlationCoefficients.begin(),min_element(correlationCoefficients.begin(),correlationCoefficients.end()));

    //正しくサブピクセル推定するために、最小となった要素の次の要素を計算し直す
    {
        double sum = 0.0;
        for(int32_t i=0; i<estimatedAngularVelocity.size();i++){
            sum +=   abs(angularVelocity(i+minPosition+1)[0]-estimatedAngularVelocity[i][0])
                    + abs(angularVelocity(i+minPosition+1)[1]-estimatedAngularVelocity[i][1])
                    + abs(angularVelocity(i+minPosition+1)[2]-estimatedAngularVelocity[i][2]);
            //            double diff = abs(angularVelocity(i+minPosition+1)[0]-estimatedAngularVelocity[i][0])
            //                        + abs(angularVelocity(i+minPosition+1)[1]-estimatedAngularVelocity[i][1]);
            //                        + abs(angularVelocity(i+minPosition+1)[2]-estimatedAngularVelocity[i][2]);
            //            sum += (diff < 0.05) ? diff : 0.05;
        }
        correlationCoefficients[minPosition+1] = sum;
    }

    t2 = std::chrono::system_clock::now() ;
    // 処理の経過時間
    elapsed = t2 - t1 ;
    std::cout << "Elapsed time@search minimum: " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << " ms\n";


    //最小値サブピクセル推定
    double subframeOffset;
    if(minPosition == 0){	//位置が最初のフレームで一致している場合
        subframeOffset = 0.0;
    }else if(minPosition == (lengthDiff-1)){//末尾
        subframeOffset = (double)(lengthDiff -1);
    }else{					//その他
        subframeOffset = -(correlationCoefficients[minPosition+1]-correlationCoefficients[minPosition-1])/(2*correlationCoefficients[minPosition-1]-4*correlationCoefficients[minPosition]+2*correlationCoefficients[minPosition+1]);
    }

    //    minPosition += 30;//マジックナンバーｗｗｗｗ

    cout << "minPosition" << minPosition << endl;
    cout << "subframe minposition :" << minPosition+subframeOffset << endl;

    auto angularVelocity_double = [&angularVelocityIn60Hz, Tvideo, Tav](double frame){
        //        double dframe = frame * Tav / Tvideo;
        double dframe = frame *  Tvideo / Tav;
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
    ////試行錯誤的に入れ替える

    //同期が取れている角速度を出力する関数を定義
    auto angularVelocitySync = [&angularVelocityIn60Hz, Tvideo, Tav, minPosition, subframeOffset](int32_t frame){
        //        double dframe = (frame + minPosition + subframeOffset) * Tav / Tvideo;
        double dframe = (frame + minPosition + subframeOffset) * Tvideo / Tav;
        int i = floor(dframe);
        double decimalPart = dframe - (double)i;
        //領域外にはみ出した時は、末端の値で埋める
        if(i<0){
            return angularVelocityIn60Hz[0];
        }else if(angularVelocityIn60Hz.size()<=(i+1)){
            return angularVelocityIn60Hz.back();
        }else{
            return angularVelocityIn60Hz[i]*(1.0-decimalPart)+angularVelocityIn60Hz[i+1]*decimalPart;
        }
    };

    if(0){
        double sum = 0.0;
        for(int32_t i=0; i<estimatedAngularVelocity.size();i++){
            sum +=   abs(angularVelocitySync(i)[0]-estimatedAngularVelocity[i][0])
                    + abs(angularVelocitySync(i)[1]-estimatedAngularVelocity[i][1])
                    + abs(angularVelocitySync(i)[2]-estimatedAngularVelocity[i][2]);
        }
        cout << " Sync correlationCoefficients:" << sum << endl;
    }

    //FIRフィルタ係数の読み込み
    //txtファイルの中身は、FIR(Finite Impluse Response)のローパスフィルタの係数である
    char coeffs[][12] = {	//!<フィルタ係数のファイル名
                            "coeff00.txt",
                            "coeff01.txt",
                            "coeff02.txt",
                            "coeff03.txt",
                            "coeff04.txt",
                            "coeff05.txt",
                            "coeff06.txt",
                            "coeff07.txt",
                            "coeff08.txt",
                            "coeff09.txt",
                            "coeff10.txt",
                            "coeff11.txt",
                        };

    std::vector<std::vector<double>> FIRcoeffs;
//    int32_t lowPassFilterStrength = 3;
    for(int i=0;i<12;i++){
        std::vector<double> temp;
        if(ReadCoeff(temp,coeffs[i])){
            return 1;
        }
        FIRcoeffs.push_back(temp);
    }

    cout << "estimated AngularVelocity and angularVelocitySync" << endl;
    cout << "i,rex,rey,rez,rx,ry,rz" << endl;
    //    for(int i = 0,e=opticShift.size();i<e;++i){
    //        printf("%d,%f,%f,%f,%f,%f,%f\n",i,estimatedAngularVelocity[i][0],estimatedAngularVelocity[i][1],estimatedAngularVelocity[i][2],angularVelocitySync(i)[0],angularVelocitySync(i)[1],angularVelocitySync(i)[2]);
    //    }

    //平滑済みクォータニオンの計算//////////////////////////
    vector<quaternion<double>> angleQuaternion;
    angleQuaternion.push_back(quaternion<double>(1,0,0,0));

    //FIRフィルタに食わせやすいように位置を合わせて角度を計算する
    int32_t halfLength = floor(FIRcoeffs[lowPassFilterStrength].size()/2);
    for(int frame=-halfLength,e=halfLength;frame<e;frame++){
        cout << "frame:" << frame << "av:" << angularVelocitySync(frame) << endl;
        angleQuaternion.push_back(angleQuaternion.back()*RotationQuaternion(angularVelocitySync(frame)*Tvideo));
        angleQuaternion.back() = angleQuaternion.back() * (1.0 / norm(angleQuaternion.back()));
    }

    if(0){
        for(int i=0;i<halfLength-1;++i){
            cout << "i:" << i << " 60Hz:" << angularVelocityIn60Hz[i] << endl;
        }
        for(int i=0;i<angleQuaternion.size();i++){
            cout << "i:" << i << " " << Quaternion2Vector( angleQuaternion[i]) << endl;
        }
    }
    //    printf("p1:%lu\n",angleQuaternion.size());

    //計算した角度をhyouzi
    if(0){
        for(auto el:angleQuaternion){
            static int cn = 0;
            printf("%d ",cn++);
            cout << Quaternion2Vector(el) << endl;
        }
    }

    quaternion<double> prevDiffAngleQuaternion;
    quaternion<double> currDiffAngleQuaternion;
    quaternion<double> nextDiffAngleQuaternion;

    quaternion<double> smoothedAngleQuaternion;
    {   //IIR平滑化
        cv::Vec3d prevVec = Quaternion2Vector(angleQuaternion[0]);
        cv::Vec3d sum(0.0, 0.0, 0.0);
        for(int32_t j=0,f=FIRcoeffs[lowPassFilterStrength].size();j<f;++j){
            cv::Vec3d curVec = Quaternion2Vector(angleQuaternion[j],prevVec);
            sum += FIRcoeffs[lowPassFilterStrength][j]*curVec;
            prevVec = curVec;
        }
        smoothedAngleQuaternion = Vector2Quaternion<double>(sum);
    }

    prevDiffAngleQuaternion = conj(smoothedAngleQuaternion)*angleQuaternion[halfLength];
    currDiffAngleQuaternion = conj(smoothedAngleQuaternion)*angleQuaternion[halfLength];


    cv::Mat buff(textureSize.height,textureSize.width,CV_8UC3);//テクスチャ用Matを準備
    //    img.copyTo(buff(cv::Rect(0,0,img.cols,img.rows)));




    // Initialise GLFW
    if( !glfwInit() )
    {
        fprintf( stderr, "Failed to initialize GLFW\n" );
        getchar();
        return -1;
    }

#define TEST2D

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef TEST2D
    glfwWindowHint( GLFW_VISIBLE, 0 );//オフスクリーンレンダリング。
#endif

//    glfwWindowHint( GLFW_VISIBLE, 0 );//オフスクリーンレンダリング。


    // Open a window and create its OpenGL context

#ifndef TEST2D
    window = glfwCreateWindow( 1920, 1080, "Tutorial 0 - Keyboard and Mouse", glfwGetPrimaryMonitor(), NULL);
#else
    window = glfwCreateWindow( 1920, 1080, "Tutorial 0 - Keyboard and Mouse", NULL, NULL);
#endif
    if( window == NULL ){
        fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
        getchar();
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Initialize GLEW
    glewExperimental = true; // Needed for core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        getchar();
        glfwTerminate();
        return -1;
    }
#ifndef TEST2D
    glfwIconifyWindow(window);
#endif
//    glfwHideWindow(window);

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
    // Hide the mouse and enable unlimited mouvement
    //        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Set the mouse at the center of the screen
    glfwPollEvents();
    //    glfwSetCursorPos(window, 1024/2, 768/2);

    // Dark blue background
    glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);

    // Cull triangles which normal is not towards the camera
    glEnable(GL_CULL_FACE);

    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    // Create and compile our GLSL program from the shaders
    GLuint programID = LoadShaders( "TransformVertexShader.vertexshader", "TextureFragmentShader.fragmentshader" );

    // Get a handle for our "MVP" uniform
    GLuint MatrixID = glGetUniformLocation(programID, "MVP");


#ifndef TEST2D
    // Create one OpenGL texture
    GLuint textureID_0;
    glGenTextures(1, &textureID_0);
    //OpenGLに「これから、テクスチャ識別子idに対して指示を与えます」と指示
    glBindTexture(GL_TEXTURE_2D,textureID_0);
    //テクスチャをここで作成
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,buff.cols,buff.rows,0,GL_BGR,GL_UNSIGNED_BYTE,buff.data);
#else
    // The framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.
    GLuint FramebufferName = 0;
    glGenFramebuffers(1, &FramebufferName);
    glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);

    // The texture we're going to render to
    GLuint renderedTexture;

    GLuint textures[2];
    glGenTextures(2, textures);
    renderedTexture = textures[0];
    GLuint textureID_0 = textures[1];

    glBindTexture(GL_TEXTURE_2D,textureID_0);
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,buff.cols,buff.rows,0,GL_BGR,GL_UNSIGNED_BYTE,buff.data);
#endif


    static const GLfloat border[] = { 0.0, 0.0, 0.0, 0.0 };//背景色
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);//テクスチャの境界色
    //テクスチャの繰り返しの設定
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

#ifdef TEST2D
    // "Bind" the newly created texture : all future texture functions will modify this texture
    glBindTexture(GL_TEXTURE_2D, renderedTexture);

    // Give an empty image to OpenGL ( the last "0" means "empty" )
    glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, imageSize.width, imageSize.height, 0,GL_RGB, GL_UNSIGNED_BYTE, 0);

    // Poor filtering
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    // Set "renderedTexture" as our colour attachement #0
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, renderedTexture, 0);

    // Set the list of draw buffers.
    GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, DrawBuffers); // "1" is the size of DrawBuffers

    // Always check that our framebuffer is ok
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE){
        return false;
    }
#endif

    // Load the texture
    //        GLuint Texture = loadDDS("uvtemplate.DDS");

    // Get a handle for our "myTextureSampler" uniform
    GLuint TextureID  = glGetUniformLocation(programID, "myTextureSampler");


    std::vector<GLfloat> vecTexture;
    for(int j=0;j<division_y;++j){							//jは終了の判定が"<"であることに注意
        double v	= (double)j/division_y*imageSize.height;
        double v1	= (double)(j+1)/division_y*imageSize.height;
        for(int i=0;i<division_x;++i){
            double u	= (double)i/division_x*imageSize.width;
            double u1	= (double)(i+1)/division_x*imageSize.width;
            //OpenGL側へ送信するテクスチャの頂点座標を準備
            vecTexture.push_back((GLfloat)u/textureSize.width);//x座標
            vecTexture.push_back((GLfloat)v/textureSize.height);//y座標
            vecTexture.push_back((GLfloat)u1/textureSize.width);//x座標
            vecTexture.push_back((GLfloat)v/textureSize.height);//y座標
            vecTexture.push_back((GLfloat)u/textureSize.width);//x座標
            vecTexture.push_back((GLfloat)v1/textureSize.height);//y座標


            vecTexture.push_back((GLfloat)u/textureSize.width);//x座標
            vecTexture.push_back((GLfloat)v1/textureSize.height);//y座標
            vecTexture.push_back((GLfloat)u1/textureSize.width);//x座標
            vecTexture.push_back((GLfloat)v/textureSize.height);//y座標
            vecTexture.push_back((GLfloat)u1/textureSize.width);//x座標
            vecTexture.push_back((GLfloat)v1/textureSize.height);//y座標


        }
    }

    std::vector<GLfloat> vecVtx(vecTexture.size());					//頂点座標

    GLuint vertexbuffer;
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    //    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);
    glBufferData(GL_ARRAY_BUFFER, vecVtx.size()*sizeof(GLfloat), vecVtx.data(), GL_DYNAMIC_DRAW);

    GLuint uvbuffer;
    glGenBuffers(1, &uvbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
    //    glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data), g_uv_buffer_data, GL_STATIC_DRAW);
    glBufferData(GL_ARRAY_BUFFER, vecTexture.size()*sizeof(GLfloat), vecTexture.data(), GL_STATIC_DRAW);

    /*    cout << "vecVtx" << endl;
    for(auto it=vecVtx.begin(),e=vecVtx.end();it!=e;it+=2) cout << *it << "," << *(it+1)  << endl;
    cout << "vecTexture" << endl;
    for(auto it=vecTexture.begin(),e=vecTexture.end();it!=e;it+=2) cout << *it << "," << *(it+1) << endl;*/




    //歪補正の準備
    GLuint nFxyID       = glGetUniformLocation(programID, "normalizedFocalLength");
    GLuint nCxyID       = glGetUniformLocation(programID, "normalizedOpticalCenter");
    GLuint distCoeffID  = glGetUniformLocation(programID, "distortionCoeffs");


    printf(",sx,sy,sz,ax,ay,az,dx,dy,dz,ex,ey,ez\r\n");

    //動画の位置を修正
    cv::Mat img;

    //一度動画を閉じて、seek可能版に置き換える
    int32_t e=Capture->get(CV_CAP_PROP_FRAME_COUNT);
    delete Capture;
    seekableVideoCapture sCapture(videoPass,PREFETCH_LENGTH);

    cv::namedWindow("Preview",cv::WINDOW_NORMAL);

    for(int32_t i=0;i<e;++i){


        //        cout << "i:" << i <<" POS:" << Capture->get(cv::CAP_PROP_POS_FRAMES) << endl;
        nextDiffAngleQuaternion = conj(smoothedAngleQuaternion)*angleQuaternion[halfLength+1];

        //モーションインペインティング用の位置を検索
        int32_t mipFrame = 0;//0は適するフレームがないことを示す

        std::vector<float> norms(PREFETCH_LENGTH);
        //まずすべてのnormを計算
        for(int32_t j=0;j<PREFETCH_LENGTH;++j){
            norms[j] = cv::norm(Quaternion2Vector(conj(smoothedAngleQuaternion)*angleQuaternion[j]));
        }
        mipFrame = std::distance(norms.begin(),std::min_element(norms.begin(),norms.end()))-PREFETCH_LENGTH/2;
        //        cout << "mipFrame:" << mipFrame << endl;


        //        nextDiffAngleQuaternion = conj(quaternion<double>(1,0,0,0))*angleQuaternion[halfLength];

        //試しに表示
        if(0){
            //            static int framen=0;
            cv::Vec3d s = Quaternion2Vector(smoothedAngleQuaternion);
            cv::Vec3d a = Quaternion2Vector(angleQuaternion[halfLength]);
            cv::Vec3d d = Quaternion2Vector(currDiffAngleQuaternion);
            //cv::Vec3d e = Quaternion2Vector(estimatedAngleQuaternion.back());
            //            printf("%d,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f\r\n",i,s[0],s[1],s[2],a[0],a[1],a[2],d[0],d[1],d[2],e[0],e[1],e[2]);
            printf("%d,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f\r\n",i,s[0],s[1],s[2],a[0],a[1],a[2],d[0],d[1],d[2]);
            //            currSmoothedAngleQuaternion = nextSmoothedAngleQuaternion;
            //            framen++;
        }
        //estimatedAngleQuaternion.push_back(estimatedAngleQuaternion.back()*RotationQuaternion(estimatedAngularVelocity[i]*Tvideo));

        //調整用のクォータニオンを準備
        quaternion<double> adjustmentQuaternion = Vector2Quaternion<double>(cv::Vec3d(vAngle,hAngle,0.0));
//        cout << "adjustmentQuaternion:" << adjustmentQuaternion << endl;
//        cout << "vAngle:" << vAngle << endl;
//        cout << "hAngle:" << hAngle << endl;
        getDistortUnrollingMap(prevDiffAngleQuaternion,currDiffAngleQuaternion,nextDiffAngleQuaternion,
                               division_x,division_y,rollingShutterDuration,matInvDistort, matIntrinsic, imageSize, adjustmentQuaternion,vecVtx,zoomRatio);
        //角度配列の先頭を削除
        angleQuaternion.erase(angleQuaternion.begin());
        //末尾に角度を追加
        angleQuaternion.push_back(angleQuaternion.back()*RotationQuaternion(angularVelocitySync(i+halfLength)*Tvideo));

        //IIR平滑化、次回の分を計算しておく
        cv::Vec3d prevVec = Quaternion2Vector(angleQuaternion[0]);
        cv::Vec3d sum(0.0, 0.0, 0.0);
        for(int32_t j=0,f=FIRcoeffs[lowPassFilterStrength].size();j<f;++j){
            cv::Vec3d curVec = Quaternion2Vector(angleQuaternion[j],prevVec);
            sum += FIRcoeffs[lowPassFilterStrength][j]*curVec;
            prevVec = curVec;
        }
        smoothedAngleQuaternion = Vector2Quaternion<double>(sum);


        //補正量を保存
        prevDiffAngleQuaternion = currDiffAngleQuaternion;
        currDiffAngleQuaternion = nextDiffAngleQuaternion;

#ifdef TEST2D
        // Render to our framebuffer
        glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
        glViewport(0,0,imageSize.width,imageSize.height); // Render on the whole framebuffer, complete from the lower left corner to the upper right
#endif

        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Use our shader
        glUseProgram(programID);


        glm::mat4 MVP = glm::mat4(1.0f);//動画保存用
        // Send our transformation to the currently bound shader,
        // in the "MVP" uniform
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);


        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glBufferData(GL_ARRAY_BUFFER, vecVtx.size()*sizeof(GLfloat), vecVtx.data(),GL_DYNAMIC_DRAW);



#if MULTITHREAD_CAPTURE
        {
            std::lock_guard<std::mutex> lock(mtvc.mtx);
            img = mtvc.images.front().clone();
            mtvc.images.pop_front();//先頭を削除
        }
#else
        sCapture.getFrame(i,img);
#endif

#ifdef TEST2D
        // Bind our texture in Texture Unit 0
        glActiveTexture(GL_TEXTURE0);
#endif

        // Bind our texture in Texture Unit 0
        //        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, textureID_0);//            glBindTexture(GL_TEXTURE_2D, Texture);
        glTexSubImage2D(GL_TEXTURE_2D,0,0,0,img.cols,img.rows,GL_BGR,GL_UNSIGNED_BYTE,img.data);
        //        glGenerateMipmap(GL_TEXTURE_2D);
        // Set our "myTextureSampler" sampler to user Texture Unit 0
        glUniform1i(TextureID, 0);

        //歪補正の準備
        float nfxy[] = {(float)(matIntrinsic.at<double>(0,0)/imageSize.width), (float)(matIntrinsic.at<double>(1,1)/imageSize.height)};
        glUniform2fv(nFxyID, 1, nfxy);
        float ncxy[] = {(float)(matIntrinsic.at<double>(0,2)/imageSize.width), (float)(matIntrinsic.at<double>(1,2)/imageSize.height)};
        glUniform2fv(nCxyID, 1, ncxy);
        //        float distcoeffFloat[] = {(float)(matDist.at<double>(0,0)),(float)(matDist.at<double>(0,1)),(float)(matDist.at<double>(0,2)),(float)(matDist.at<double>(0,3))};
        float distcoeffFloat[] = {(float)(matInvDistort.at<double>(0,0)),(float)(matInvDistort.at<double>(0,1)),(float)(matInvDistort.at<double>(0,2)),(float)(matInvDistort.at<double>(0,3))};
        glUniform4fv(distCoeffID, 1, distcoeffFloat);

        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(
                    0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
                    2,                  // size
                    GL_FLOAT,           // type
                    GL_FALSE,           // normalized?
                    0,                  // stride
                    (void*)0            // array buffer offset
                    );

        // 2nd attribute buffer : UVs
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
        glVertexAttribPointer(
                    1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
                    2,                                // size : U+V => 2
                    GL_FLOAT,                         // type
                    GL_FALSE,                         // normalized?
                    0,                                // stride
                    (void*)0                          // array buffer offset
                    );

        // Draw the triangle !
        glDrawArrays(GL_TRIANGLES, 0, vecVtx.size()*2); // 12*3 indices starting at 0 -> 12 triangles





#if 0   //モーションインペインティング
        if(abs(mipFrame)>6){
            t1 = std::chrono::system_clock::now() ;
            int32_t divNum = 5;
            for(int32_t k=mipFrame/divNum;abs(k)<=abs(mipFrame);k+=mipFrame/divNum){



                quaternion<double> mipDiffAngleQuaternion = conj(smoothedAngleQuaternion)*angleQuaternion[halfLength+k];


                std::vector<GLfloat> vecVtx4MIP(vecTexture.size());					//頂点座標

                getDistortUnrollingMap(mipDiffAngleQuaternion,mipDiffAngleQuaternion,mipDiffAngleQuaternion,
                                       division_x,division_y,0,matInvDistort, matIntrinsic, imageSize, vecVtx4MIP,ZOOM_RATIO);

                if(sCapture.getFrameForMIP(i+k,img)){
                    //                    glDeleteBuffers(1, &vertexbuffer);
                    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
                    glBufferData(GL_ARRAY_BUFFER, vecVtx4MIP.size()*sizeof(GLfloat), vecVtx4MIP.data(),GL_DYNAMIC_DRAW);


                    glBindTexture(GL_TEXTURE_2D, textureID_0);
                    glTexSubImage2D(GL_TEXTURE_2D,0,0,0,img.cols,img.rows,GL_BGR,GL_UNSIGNED_BYTE,img.data);
                    glUniform1i(TextureID, 0);

                    //歪補正の準備
                    float nfxy[] = {(float)(matIntrinsic.at<double>(0,0)/imageSize.width), (float)(matIntrinsic.at<double>(1,1)/imageSize.height)};
                    glUniform2fv(nFxyID, 1, nfxy);
                    float ncxy[] = {(float)(matIntrinsic.at<double>(0,2)/imageSize.width), (float)(matIntrinsic.at<double>(1,2)/imageSize.height)};
                    glUniform2fv(nCxyID, 1, ncxy);
                    //        float distcoeffFloat[] = {(float)(matDist.at<double>(0,0)),(float)(matDist.at<double>(0,1)),(float)(matDist.at<double>(0,2)),(float)(matDist.at<double>(0,3))};
                    float distcoeffFloat[] = {(float)(matInvDistort.at<double>(0,0)),(float)(matInvDistort.at<double>(0,1)),(float)(matInvDistort.at<double>(0,2)),(float)(matInvDistort.at<double>(0,3))};
                    glUniform4fv(distCoeffID, 1, distcoeffFloat);

                    // 1rst attribute buffer : vertices
                    glEnableVertexAttribArray(0);
                    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
                    glVertexAttribPointer(
                                0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
                                2,                  // size
                                GL_FLOAT,           // type
                                GL_FALSE,           // normalized?
                                0,                  // stride
                                (void*)0            // array buffer offset
                                );

                    // 2nd attribute buffer : UVs
                    glEnableVertexAttribArray(1);
                    glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
                    glVertexAttribPointer(
                                1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
                                2,                                // size : U+V => 2
                                GL_FLOAT,                         // type
                                GL_FALSE,                         // normalized?
                                0,                                // stride
                                (void*)0                          // array buffer offset
                                );

                    // Draw the triangle !
                    glDrawArrays(GL_TRIANGLES, 0, vecVtx4MIP.size()*2); // 12*3 indices starting at 0 -> 12 triangles

                    glBindBuffer(GL_ARRAY_BUFFER, 0);

                    glDisableVertexAttribArray(0);
                    glDisableVertexAttribArray(1);
                }
            }
            t2 = std::chrono::system_clock::now();
            // 処理の経過時間
            elapsed = t2 - t1 ;
            std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << " ms\n";

        }

#endif

#if 1
        ////////////////////
        cv::Mat simg(imageSize,CV_8UC3);
        //~ glReadBuffer(GL_FRONT);//読み取るOpenGLのバッファを指定 GL_FRONT:フロントバッファ GL_BACK:バックバッファ


//        glReadBuffer(GL_BACK);//読み取るOpenGLのバッファを指定 GL_FRONT:フロントバッファ GL_BACK:バックバッファ
        // OpenGLで画面に描画されている内容をバッファに格納
        glReadPixels(
                    0,					//読み取る領域の左下隅のx座標
                    0,					//読み取る領域の左下隅のy座標 //0 or getCurrentWidth() - 1
                    imageSize.width,				//読み取る領域の幅
                    imageSize.height,				//読み取る領域の高さ
                    GL_BGR,				//it means GL_BGR,           //取得したい色情報の形式
                    GL_UNSIGNED_BYTE,	//読み取ったデータを保存する配列の型
                    simg.data			//ビットマップのピクセルデータ（実際にはバイト配列）へのポインタ
                    );
#else
      cv::Mat simg(textureSize,CV_8UC3);
      glGetTexImage(GL_TEXTURE_2D,0,GL_BGR,GL_UNSIGNED_BYTE,simg.data);
#endif

      glDisableVertexAttribArray(0);
      glDisableVertexAttribArray(1);

        cv::imshow("Stabilized Image2",simg);
        char key =cv::waitKey(1);

        if(outputStabilizedVideo){
            std::lock_guard<std::mutex> lock(buffer.mtx);
            buffer.images.push_back(cv::Mat());
            buffer.images.back() = simg.clone();
        }


        /////////////////////

        // Swap buffers
        //        glfwSwapBuffers(window);
        glfwPollEvents();

        if(glfwGetKey(window, GLFW_KEY_ESCAPE ) == GLFW_PRESS ||
                glfwWindowShouldClose(window) != 0 ){
            break;
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


    } // Check if the ESC key was pressed or the window was closed
    //    while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
    //           glfwWindowShouldClose(window) == 0 );

    // Cleanup VBO and shader
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteBuffers(1, &uvbuffer);
    glDeleteProgram(programID);
    glDeleteTextures(1, &TextureID);
    glDeleteVertexArrays(1, &VertexArrayID);
    //ここでTextureID_0をDeleteしなくて大丈夫？
    // Close OpenGL window and terminate GLFW
    glfwTerminate();

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
        std::cout << "音声を分離" << std::endl;
        std::string command = "ffmpeg -i " + sInputVideoPass +  " -vn -acodec copy output-audio.aac";
        system(command.c_str());
        std::cout << "音声を結合" << std::endl;
        command = "ffmpeg -i " + sInputVideoPass + "_deblured.avi -i output-audio.aac -codec copy " + sInputVideoPass + "_deblured_audio.avi";
        system(command.c_str());

        system("rm output-audio.aac");
        command = "rm " + sInputVideoPass + "_deblured.avi";
        system(command.c_str());
    }
//#endif
    return 0;
}

