#ifndef VSP_H
#define VSP_H

#include <stdio.h>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <boost/math/quaternion.hpp>
#include "settings.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
using namespace std;
using namespace boost::math;

// Include standard headers
#include <stdio.h>
#include <stdlib.h>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <glfw3.h>
//extern GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include <common/shader.hpp>
#include <common/texture.hpp>
#include <common/controls.hpp>
#include "seekablevideocapture.h"
#include "camera_information.h"

class vsp
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    float fs = 0.0;
    float fc = 0.0;
    /**
     * @brief コンストラクタ
     * @param [in]	Qa	ジャイロの角速度から計算したカメラの方向を表す回転クウォータニオン時系列データ、参照渡し
     * @param [in]	Qf	LPFを掛けて平滑化した回転クウォータニオンの時系列データ、参照渡し
     * @param [in]	m	画面の縦の分割数[ ]
     * @param [in]	n	画面の横の分割数[ ]
     * @param [in]	IK	"逆"歪係数(k1,k2,p1,p2)
     * @param [in]	matIntrinsic	カメラ行列(fx,fy,cx,cy) [pixel]
     * @param [in]	imageSize	フレーム画像のサイズ[pixel]
     * @param [in]  adjustmentQuaternion 画面方向を微調整するクォータニオン[rad]
     * @param [in]	zoom	倍率[]。拡大縮小しないなら1を指定すること。省略可
     * @param [out] error はみ出したノルムの長さ
     **/
    vsp(int32_t division_x,
        int32_t division_y,
        //        double TRollingShutter,
        //        Eigen::MatrixXd IK,
        //        Eigen::MatrixXd matIntrinsic,
        //        int32_t image_width,
        //        int32_t image_height,
        CameraInformation camera_info,
        double zoom,
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &angular_velocity,
        double T_video,
        double T_angular_velocity,
        double frame_offset,
        int32_t video_frames,
        int32_t filter_tap_length);

    vsp(int32_t division_x,
        int32_t division_y,
        //        double TRollingShutter,
        //        Eigen::MatrixXd IK,
        //        Eigen::MatrixXd matIntrinsic,
        //        int32_t image_width,
        //        int32_t image_height,
        CameraInformation camera_info,
        double zoom,
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &angular_velocity,
        double T_video,
        double T_angular_velocity,
        double frame_offset,
        int32_t video_frames,
        int32_t filter_tap_length,
        Eigen::MatrixXd &raw_angle,
        Eigen::MatrixXd &filtered_quaternion);
    void setParam(double fs, double fc);
    //    vector<double> getRow(int r);

    //    const Eigen::MatrixXd &data();

    const Eigen::MatrixXd &toQuaternion();
    //    const std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond>> &toQuaternion_vec();

    template <class T>
    void setFilterCoeff(T coeff)
    {
        filter_coeff.resize(coeff.size(), 1);
        for (int i = 0, e = coeff.size(); i < e; ++i)
        {
            filter_coeff(i, 0) = coeff[i];
        }
    }

    //    const Eigen::MatrixXd &filteredData();

    static Eigen::VectorXd getKaiserWindow(uint32_t tap_length, uint32_t alpha, bool swap = true);
    static Eigen::VectorXd getKaiserWindowWithZeros(int32_t data_length, double alpha, int32_t window_length);

    static Eigen::VectorXcd getLPFFrequencyCoeff(uint32_t N, uint32_t alpha, double fs, double fc);
    static Eigen::VectorXcd getKaiserWindowWithZerosFrequencyCoeff(int32_t data_length, double alpha, int32_t window_length);

    //    static void Angle2CLerpedFrequency(double fs, double fc, const Eigen::MatrixXd &raw_angle, Eigen::MatrixXcd &freq_vectors);
    //    static void Frequency2Angle(Eigen::MatrixXcd &frequency_vector_, Eigen::MatrixXd &angle_);
    static void MatrixXcd2VectorXd(const Eigen::MatrixXcd &src, Eigen::VectorXd &dst);
    static void VectorXd2MatrixXcd(const Eigen::VectorXd &src, Eigen::MatrixXcd &dst);

    //    const Eigen::MatrixXd &filteredDataDFT(double fs, double fc);
    //    const Eigen::MatrixXd &filteredDataDFTTimeDomainOptimize(double fs, double fc, const Eigen::MatrixXd &coeff);
    //    Eigen::MatrixXd &filteredDataDFT();
    Eigen::Quaterniond filteredQuaternion(int32_t alpha, int32_t frame /*, double fs, double fc*/);
    Eigen::MatrixXd &filteredQuaternion(uint32_t alpha /*, double fs, double fc*/);
    Eigen::MatrixXd &filteredQuaternion(Eigen::VectorXd &filter_coefficients);
    //    Eigen::Quaternion<double> toRawQuaternion(uint32_t frame);
    //    Eigen::Quaternion<double> toFilteredQuaternion(uint32_t frame);
    //    Eigen::Quaternion<double> toDiffQuaternion(uint32_t frame);
    Eigen::Quaternion<double> toDiffQuaternion2(uint32_t frame);
    Eigen::Quaternion<double> toDiffQuaternion2(int32_t filter_strength, uint32_t frame);

    /**
      * @brief cos関数で2点をなめらかに補完する関数
      * @retval numステップで補完された行列
      **/
    static Eigen::MatrixXd CLerp(Eigen::MatrixXd start, Eigen::MatrixXd end, int32_t num);

    /**
     * @brief 回転を表すクォータニオンを生成する関数
     **/
    static Eigen::Quaternion<double> RotationQuaternion(double theta, Eigen::Vector3d n);

    /**
     * @brief 微小回転を表す回転ベクトルから四元数を作る関数
     **/

    static Eigen::Quaternion<double> RotationQuaternion(Eigen::Vector3d w);

    /**
     * @param 回転を表すクォータニオンをシングルローテーションをあらわすベクトルへ変換
     **/
    template <typename T_num>
    static Eigen::Vector3d Quaternion2Vector(Eigen::Quaternion<T_num> q)
    {
        double denom = sqrt(1 - q.w() * q.w());
        if (abs(denom) < EPS_Q)
        {                                    //まったく回転しない時は０割になるので、場合分けする//TODO:
            return Eigen::Vector3d(0, 0, 0); //return zero vector
        }
        return Eigen::Vector3d(q.x(), q.y(), q.z()) * 2.0 * atan2(denom, q.w()) / denom;
    }

    /**
     * @param シングルローテーションを表すベクトルを回転を表すクォータニオンへ変換
     **/
    template <typename T_num>
    static Eigen::Quaternion<T_num> Vector2Quaternion(Eigen::Vector3d w)
    {
        double theta = w.norm(); //sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]);//回転角度を計算、normと等しい
        //0割を回避するためにマクローリン展開
        if (theta > EPS_Q)
        {
            Eigen::Vector3d n = w.normalized(); //w * (1.0/theta);//単位ベクトルに変換
            //            double sin_theta_2 = sin(theta*0.5);
            //            return Eigen::Quaternion<T_num>(cos(theta*0.5),n[0]*sin_theta_2,n[1]*sin_theta_2,n[2]*sin_theta_2);
            Eigen::VectorXd n_sin_theta_2 = n * sin(theta * 0.5);
            return Eigen::Quaternion<T_num>(cos(theta * 0.5), n_sin_theta_2[0], n_sin_theta_2[1], n_sin_theta_2[2]);
        }
        else
        {
            return Eigen::Quaternion<T_num>(1.0, 0.5 * w[0], 0.5 * w[1], 0.5 * w[2]);
        }
    }

    /**
     * @brief AngleVectorをQuarternionに変換します。
     */
    //    template <typename T_num> static Eigen::Quaternion<T_num> Vector2Quaternion(Eigen::VectorXd &angle, int32_t row){
    //        return Vector2Quaternion<T_num>(angle.row(row).transpose());
    //    }

    //    Eigen::Quaternion<T_num> toDiffQuaternion()

    /**
     * @param 回転を表すクォータニオンをシングルローテーションを表すベクトルへ変換。前回算出したベクトルを引数として受け取ることで、アンラッピングする。
     * */
    //    template <typename T_num> static Eigen::Vector3d Quaternion2Vector(Eigen::Quaternion<T_num> q, Eigen::Vector3d &prev){
    //        double denom = sqrt(1-q.w()*q.w());
    //        if(denom==0.0){//まったく回転しない時は０割になるので、場合分けする
    //            return Eigen::Vector3d(0,0,0);//return zero vector
    //        }
    //        double theta = 2.0 * atan2(denom,q.w());
    //        double prev_theta = prev.norm()/2;
    //        double diff = theta - prev_theta;
    //        theta -= 2.0*M_PI*(double)(static_cast<int>(diff/(2.0*M_PI)));//マイナスの符号に注意
    //        //~ printf("Theta_2:%4.3f sc:%d\n",theta_2,static_cast<int>(diff/(2.0*M_PI)));
    //        if(static_cast<int>(diff/(2.0*M_PI))!=0){
    //            printf("\n###########Unwrapping %d\n",static_cast<int>(diff/(2.0*M_PI)));
    //        }
    ////        std::cout <<  ", q:" << (Eigen::Vector3d(q.x(),q.y(),q.z())*2.0*theta_2/denom).transpose() << std::endl;
    //        Eigen::Vector3d n(q.x(),q.y(),q.z());
    //        n /= n.norm();
    //        std::cout <<  "theta:" << theta << ", q:" << Eigen::Vector4d(q.w(),q.x(),q.y(),q.z()).transpose() <<
    //                      " denom:" << denom <<
    //                      " n:" << n.transpose() << std::endl;

    //        return Eigen::Vector3d(q.x(),q.y(),q.z())*theta/denom;
    //    }

    /**
     * @param 回転を表すクォータニオンから回転を表す行列を生成
     **/
    template <typename T_num>
    static void Quaternion2Matrix(Eigen::Quaternion<T_num> &q, Eigen::MatrixXd &det)
    {
        det = q.matrix();
    }

    /** @brief 補正前の画像座標から、補正後のポリゴンの頂点を作成
     * @param [in]	Qa	ジャイロの角速度から計算したカメラの方向を表す回転クウォータニオン時系列データ、参照渡し
     * @param [in]	Qf	LPFを掛けて平滑化した回転クウォータニオンの時系列データ、参照渡し
     * @param [in]	m	画面の縦の分割数[ ]
     * @param [in]	n	画面の横の分割数[ ]
     * @param [in]	IK	"逆"歪係数(k1,k2,p1,p2)
     * @param [in]	matIntrinsic	カメラ行列(fx,fy,cx,cy) [pixel]
     * @param [in]	imageSize	フレーム画像のサイズ[pixel]
     * @param [in]  adjustmentQuaternion 画面方向を微調整するクォータニオン[rad]
     * @param [out]	vecPorigonn_uv	OpenGLのポリゴン座標(u',v')座標(-1~1)の組、歪補正後の画面を分割した時の一つ一つのポリゴンの頂点の組
     * @param [in]	zoom	倍率[]。拡大縮小しないなら1を指定すること。省略可
     * @retval true:成功 false:折り返し発生で失敗
     **/
    template <typename _Tp, typename _Tx>
    bool getDistortUnrollingContour(
        Eigen::Quaternion<_Tp> &prevAngleQuaternion,
        Eigen::Quaternion<_Tp> &currAngleQuaternion,
        Eigen::Quaternion<_Tp> &nextAngleQuaternion,
        //            uint32_t division_x,
        //            uint32_t division_y,
        //            double TRollingShutter,
        //            Eigen::MatrixXd &IK,
        //            Eigen::MatrixXd &matIntrinsic,
        //            uint32_t camera_info_.width_,
        //            uint32_t camera_info_.height_,
        std::vector<_Tx> &vecPorigonn_uv
        //            double zoom
    )
    {

        bool retval = true;

        //手順
        //1.補正前画像を分割した時の分割点の座標(pixel)を計算
        //2.1の座標を入力として、各行毎のW(t1,t2)を計算
        //3.補正後の画像上のポリゴン座標(pixel)を計算、歪み補正も含める
        vecPorigonn_uv.clear();

        //top
        //    for(int j=0;j<=division_y;++j){
        {
            int j = 0;
            //W(t1,t2)を計算
            Eigen::MatrixXd R;
            //1
            double v = (double)j / division_y * camera_info_.height_;

            double exposureTimingInEachRow = camera_info_.line_delay_ * v; //ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]

            Eigen::Quaternion<double> slerpedAngleQuaternion;
            if (exposureTimingInEachRow >= 0)
            {
                slerpedAngleQuaternion = currAngleQuaternion.slerp(exposureTimingInEachRow, nextAngleQuaternion);
            }
            else
            {
                slerpedAngleQuaternion = prevAngleQuaternion.slerp(1.0 + exposureTimingInEachRow, currAngleQuaternion);
            }
            Quaternion2Matrix(slerpedAngleQuaternion, R);
            for (int i = 0; i <= division_x; ++i)
            {
                double u = (double)i / division_x * camera_info_.width_;
                //後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
                Eigen::Vector3d p;
                p << (u - camera_info_.cx_) / camera_info_.fx_, (v - camera_info_.cy_) / camera_info_.fy_, 1.0; //1のポリゴン座標に、K^-1を掛けた結果の３x１行列
                //2
                Eigen::MatrixXd XYW = R * p; //inv()なし

                if (XYW(2, 0) < 0.0)
                {
                    retval = false;
                }

                double x1 = XYW(0, 0) / XYW(2, 0);
                double y1 = XYW(1, 0) / XYW(2, 0);

                double r = sqrt(x1 * x1 + y1 * y1);

                double x2 = x1 * (1.0 + camera_info_.inverse_k1_ * r * r + camera_info_.inverse_k2_ * r * r * r * r) + 2.0 * camera_info_.inverse_p1_ * x1 * y1 + camera_info_.inverse_p2_ * (r * r + 2.0 * x1 * x1);
                double y2 = y1 * (1.0 + camera_info_.inverse_k1_ * r * r + camera_info_.inverse_k2_ * r * r * r * r) + camera_info_.inverse_p1_ * (r * r + 2.0 * y1 * y1) + 2.0 * camera_info_.inverse_p2_ * x1 * y1;
                //変な折り返しを防止
                if ((pow(x2 - x1, 2) > 1.0) || (pow(y2 - y1, 2) > 1.0))
                {
                    //                printf("折り返し防止\r\n");
                    x2 = x1;
                    y2 = y1;
                }
                vecPorigonn_uv.push_back(x2 * camera_info_.fx_ * zoom / camera_info_.width_ * 2.0);//TODO:ここ多分間違えている。修正。
                vecPorigonn_uv.push_back(y2 * camera_info_.fy_ * zoom / camera_info_.height_ * 2.0);
            }
        }

        //middle
        for (int j = 1; j < division_y; ++j)
        {
            //W(t1,t2)を計算
            Eigen::MatrixXd R;
            //1
            double v = (double)j / division_y * camera_info_.height_;

            double exposureTimingInEachRow = camera_info_.line_delay_ * v ; //ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]

            Eigen::Quaternion<double> slerpedAngleQuaternion;
            if (exposureTimingInEachRow >= 0)
            {
                slerpedAngleQuaternion = currAngleQuaternion.slerp(exposureTimingInEachRow, nextAngleQuaternion);
            }
            else
            {
                slerpedAngleQuaternion = prevAngleQuaternion.slerp(1.0 + exposureTimingInEachRow, currAngleQuaternion);
            }
            Quaternion2Matrix(slerpedAngleQuaternion, R);
            for (int i = 0; i <= division_x; i += division_x)
            {
                double u = (double)i / division_x * camera_info_.width_;
                //後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
                Eigen::Vector3d p;
                p << (u - camera_info_.cx_) / camera_info_.fx_, (v - camera_info_.cy_) / camera_info_.fy_, 1.0; //1のポリゴン座標に、K^-1を掛けた結果の３x１行列
                //2
                Eigen::MatrixXd XYW = R * p; //inv()なし

                if (XYW(2, 0) < 0.0)
                {
                    retval = false;
                }

                double x1 = XYW(0, 0) / XYW(2, 0);
                double y1 = XYW(1, 0) / XYW(2, 0);

                double r = sqrt(x1 * x1 + y1 * y1);

                double x2 = x1 * (1.0 + camera_info_.inverse_k1_ * r * r + camera_info_.inverse_k2_ * r * r * r * r) + 2.0 * camera_info_.inverse_p1_ * x1 * y1 + camera_info_.inverse_p2_ * (r * r + 2.0 * x1 * x1);
                double y2 = y1 * (1.0 + camera_info_.inverse_k1_ * r * r + camera_info_.inverse_k2_ * r * r * r * r) + camera_info_.inverse_p1_ * (r * r + 2.0 * y1 * y1) + 2.0 * camera_info_.inverse_p2_ * x1 * y1;
                //変な折り返しを防止
                if ((pow(x2 - x1, 2) > 1.0) || (pow(y2 - y1, 2) > 1.0))
                {
                    //                printf("折り返し防止\r\n");
                    x2 = x1;
                    y2 = y1;
                }
                vecPorigonn_uv.push_back(x2 * camera_info_.fx_ * zoom / camera_info_.width_ * 2.0);
                vecPorigonn_uv.push_back(y2 * camera_info_.fy_ * zoom / camera_info_.height_ * 2.0);
            }
        }

        //bottom
        {
            int j = division_y;
            //W(t1,t2)を計算
            Eigen::MatrixXd R;
            //1
            double v = (double)j / division_y * camera_info_.height_;

            double exposureTimingInEachRow = camera_info_.line_delay_ * v; //ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]

            Eigen::Quaternion<double> slerpedAngleQuaternion;
            if (exposureTimingInEachRow >= 0)
            {
                slerpedAngleQuaternion = currAngleQuaternion.slerp(exposureTimingInEachRow, nextAngleQuaternion);
            }
            else
            {
                slerpedAngleQuaternion = prevAngleQuaternion.slerp(1.0 + exposureTimingInEachRow, currAngleQuaternion);
            }
            Quaternion2Matrix(slerpedAngleQuaternion, R);
            for (int i = 0; i <= division_x; ++i)
            {
                double u = (double)i / division_x * camera_info_.width_;
                //後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
                Eigen::Vector3d p;
                p << (u - camera_info_.cx_) / camera_info_.fx_, (v - camera_info_.cy_) / camera_info_.fy_, 1.0; //1のポリゴン座標に、K^-1を掛けた結果の３x１行列
                //2
                Eigen::MatrixXd XYW = R * p; //inv()なし

                if (XYW(2, 0) < 0.0)
                {
                    retval = false;
                }

                double x1 = XYW(0, 0) / XYW(2, 0);
                double y1 = XYW(1, 0) / XYW(2, 0);

                double r = sqrt(x1 * x1 + y1 * y1);

                double x2 = x1 * (1.0 + camera_info_.inverse_k1_ * r * r + camera_info_.inverse_k2_ * r * r * r * r) + 2.0 * camera_info_.inverse_p1_ * x1 * y1 + camera_info_.inverse_p2_ * (r * r + 2.0 * x1 * x1);
                double y2 = y1 * (1.0 + camera_info_.inverse_k1_ * r * r + camera_info_.inverse_k2_ * r * r * r * r) + camera_info_.inverse_p1_ * (r * r + 2.0 * y1 * y1) + 2.0 * camera_info_.inverse_p2_ * x1 * y1;
                //変な折り返しを防止
                if ((pow(x2 - x1, 2) > 1.0) || (pow(y2 - y1, 2) > 1.0))
                {
                    //                printf("折り返し防止\r\n");
                    x2 = x1;
                    y2 = y1;
                }
                vecPorigonn_uv.push_back(x2 * camera_info_.fx_ * zoom / camera_info_.width_ * 2.0);
                vecPorigonn_uv.push_back(y2 * camera_info_.fy_ * zoom / camera_info_.height_ * 2.0);
            }
        }

        return retval;
    }

    

    template <typename _Tx>
    bool getDistortUnrollingMapQuaternion(
        int32_t frame,
        std::vector<_Tx> &vecPorigonn_uv)
    {

        Eigen::Quaternion<double> prevAngleQuaternion = this->toDiffQuaternion2(frame);
        Eigen::Quaternion<double> currAngleQuaternion = this->toDiffQuaternion2(frame + 1); //インデックスの付け方が気持ち悪い。TODO:何とかする
        Eigen::Quaternion<double> nextAngleQuaternion = this->toDiffQuaternion2(frame + 2);
        bool retval = true;

        //手順
        //1.補正前画像を分割した時の分割点の座標(pixel)を計算
        //2.1の座標を入力として、各行毎のW(t1,t2)を計算
        //3.補正後の画像上のポリゴン座標(pixel)を計算、歪み補正も含める

        vecPorigonn_uv.clear();
        Eigen::MatrixXd map_x = Eigen::MatrixXd::Zero(division_y + 1, division_x + 1);
        Eigen::MatrixXd map_y = Eigen::MatrixXd::Zero(division_y + 1, division_x + 1);
        for (int j = 0; j <= division_y; ++j)
        {
            //W(t1,t2)を計算
            Eigen::MatrixXd R;
            //1
            double v = (double)j / division_y * camera_info_.height_;

            double exposureTimingInEachRow = camera_info_.line_delay_ * v; //ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]

            Eigen::Quaternion<double> slerpedAngleQuaternion;
            if (exposureTimingInEachRow >= 0)
            {
                slerpedAngleQuaternion = currAngleQuaternion.slerp(exposureTimingInEachRow, nextAngleQuaternion);
            }
            else
            {
                slerpedAngleQuaternion = prevAngleQuaternion.slerp(1.0 + exposureTimingInEachRow, currAngleQuaternion);
            }
            Quaternion2Matrix(slerpedAngleQuaternion, R);
            for (int i = 0; i <= division_x; ++i)
            {
                double u = (double)i / division_x * camera_info_.width_;
                //後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
                Eigen::Vector3d p;
                p << (u - camera_info_.cx_) / camera_info_.fx_, (v - camera_info_.cy_) / camera_info_.fy_, 1.0; //1のポリゴン座標に、K^-1を掛けた結果の３x１行列

                //2
                Eigen::MatrixXd XYW;
                if(0){
                    XYW = R * p; //inv()なし
                }else{
                    XYW = p;
                }

                if (XYW(2, 0) < 0.0)
                {
                    retval = false;
                }

                double x1 = XYW(0, 0) / XYW(2, 0);
                double y1 = XYW(1, 0) / XYW(2, 0);

                double r = sqrt(x1 * x1 + y1 * y1);

                double x2 = x1 * (1.0 + camera_info_.inverse_k1_ * r * r + camera_info_.inverse_k2_ * r * r * r * r) + 2.0 * camera_info_.inverse_p1_ * x1 * y1 + camera_info_.inverse_p2_ * (r * r + 2.0 * x1 * x1);
                double y2 = y1 * (1.0 + camera_info_.inverse_k1_ * r * r + camera_info_.inverse_k2_ * r * r * r * r) + camera_info_.inverse_p1_ * (r * r + 2.0 * y1 * y1) + 2.0 * camera_info_.inverse_p2_ * x1 * y1;
                //変な折り返しを防止
                if ((pow(x2 - x1, 2) > 1.0) || (pow(y2 - y1, 2) > 1.0))
                {
                    //                printf("折り返し防止\r\n");
                    x2 = x1;
                    y2 = y1;
                }

                if(0){

                }else{
                    p << x2, y2,1.0;
                    XYW = R * p;
                    x2 = XYW(0, 0) / XYW(2, 0);
                    y2 = XYW(1, 0) / XYW(2, 0);
                }

                //                vecPorigonn_uv.push_back(x2*fx*zoom/camera_info_.width_*2.0);
                //                vecPorigonn_uv.push_back(y2*fy*zoom/camera_info_.height_*2.0);
                map_x(j, i) = x2 * camera_info_.fx_ * zoom / camera_info_.width_ * 2.0;
                map_y(j, i) = y2 * camera_info_.fy_ * zoom / camera_info_.height_ * 2.0;
            }
        }

        //3.ポリゴン座標をOpenGLの関数に渡すために順番を書き換える
        vecPorigonn_uv.clear();
        for (int j = 0; j < division_y; ++j)
        { //jは終了の判定が"<"であることに注意
            for (int i = 0; i < division_x; ++i)
            {
                //GL_TRIANGLESでGL側へ送信するポリゴンの頂点座標を準備
                vecPorigonn_uv.push_back(map_x(j, i));     //x座標
                vecPorigonn_uv.push_back(map_y(j, i));     //y座標
                vecPorigonn_uv.push_back(map_x(j, i + 1)); //x座標
                vecPorigonn_uv.push_back(map_y(j, i + 1)); //y座標
                vecPorigonn_uv.push_back(map_x(j + 1, i)); //x座標
                vecPorigonn_uv.push_back(map_y(j + 1, i)); //y座標

                vecPorigonn_uv.push_back(map_x(j + 1, i));     //x座標
                vecPorigonn_uv.push_back(map_y(j + 1, i));     //y座標
                vecPorigonn_uv.push_back(map_x(j, i + 1));     //x座標
                vecPorigonn_uv.push_back(map_y(j, i + 1));     //y座標
                vecPorigonn_uv.push_back(map_x(j + 1, i + 1)); //x座標
                vecPorigonn_uv.push_back(map_y(j + 1, i + 1)); //y座標
            }
        }

        return retval;
    }

    /**
     * @brief ワープした時に欠けがないかチェックします
     * @retval false:欠けあり true:ワープが良好
     **/
    template <typename _Tp>
    static bool isPerfectWarp(vector<_Tp> &contour)
    {
        for (int i = 0; i < contour.size(); i += 2)
        {
            if ((abs(contour[i]) < 1.0) && (abs(contour[i + 1]) < 1.0))
            {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief 画面の欠けを生み出している回転ベクトルのオーバーした長さを返す
     **/
    Eigen::VectorXd getRollingVectorError();

    /**
      * @brief 同期が取れている角速度を出力
      **/
    Eigen::Vector3d angularVelocitySync(int32_t frame);

    template <typename _Tp, typename _Alloc = std::allocator<_Tp>>
    static void angularVelocityCoordinateTransformer(std::vector<_Tp, _Alloc> &angular_velocity, const Eigen::Quaterniond &rotation)
    {
        Eigen::Quaterniond avq;
        avq.w() = 0.0;
        for (auto &el : angular_velocity)
        {
            avq.x() = el[0];
            avq.y() = el[1];
            avq.z() = el[2];
            avq = rotation * avq * rotation.conjugate(); //Rotate angular velocity vector
            el[0] = avq.x();
            el[1] = avq.y();
            el[2] = avq.z();
        }
    }

    int init_opengl(cv::Size textureSize);
    int stop_opengl();
    int spin_once(int frame, cv::VideoCapture &capture, cv::Mat &simg);
    bool ok();

    Eigen::VectorXd calculateFilterCoefficientsWithoutBlackSpaces(int32_t minimum_filter_strength, int32_t maximum_filter_strength);
    uint32_t bisectionMethod(int32_t frame, int32_t minimum_filter_strength, int32_t maximum_filter_strength, int max_iteration = 1000, uint32_t eps = 1);
    bool hasBlackSpace(int32_t filter_strength, int32_t frame);
    void gradientLimit(Eigen::VectorXd &input);
    void setMaximumGradient(double value);
    Eigen::MatrixXd &getRawQuaternion();
    Eigen::MatrixXd &getFilteredQuaternion();

  private:
    enum KEY
    {
        KEY_SIDEBYSIDE = '1',
        KEY_ORIGINAL = '2',
        KEY_STABILIZED = '3',
        KEY_QUIT = 'q'
    };

    //    Eigen::MatrixXd raw_angle;//TODO:REMOVE
    Eigen::MatrixXd raw_quaternion;
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> raw_quaternion_with_margin;
    //    Eigen::MatrixXd filtered_angle;//TODO:REMOVE
    Eigen::MatrixXd filtered_quaternion_;
    Eigen::VectorXd filter_coeff;
    bool is_filtered;
    bool quaternion_is_filtered = false;
    int32_t division_x = 9;
    int32_t division_y = 9;
    //    double camera_info_.line_delay_ = 0.0;
    //    Eigen::MatrixXd IK = Eigen::MatrixXd::Zero(1,4);
    //    Eigen::MatrixXd matIntrinsic = Eigen::MatrixXd::Identity(3,3);
    //    int32_t image_width=1920;
    //    int32_t image_height=1080;
    const CameraInformation camera_info_;
    double zoom = 1.0;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> angular_velocity;
    double T_video = 1.0 / 30.0;
    double T_angular_velocity = 1.0 / 60.0;
    double frame_offset = 0;
    int32_t video_frames = 0;
    int32_t filter_tap_length = 0;

    char key = '1';

    //OpenGL
    cv::Size textureSize;
    cv::Mat buff;
    GLuint vertexbuffer;
    GLuint uvbuffer;
    GLuint programID;
    GLuint TextureID;
    GLuint VertexArrayID;
    std::vector<GLfloat> vecVtx; //頂点座標
    GLuint FramebufferName = 0;
    GLuint MatrixID;
    cv::Mat img;
    GLuint textureID_0;
    GLuint nFxyID;
    GLuint nCxyID;
    GLuint distCoeffID;

    //Removin black space
    double maximum_gradient_;
};

#endif // VSP_H
