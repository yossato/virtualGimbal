#ifndef VSP_H
#define VSP_H

#include <stdio.h>
#include <iostream>
#include <boost/math/quaternion.hpp>
#include "settings.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
using namespace std;
using namespace boost::math;

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
    template <class T> vsp(vector<Eigen::Quaternion<T>> &angle_quaternion,
                           int32_t division_x,
                           int32_t division_y,
                           double TRollingShutter,
                           Eigen::MatrixXd IK,
                           Eigen::MatrixXd matIntrinsic,
                           int32_t image_width,
                           int32_t image_height,
                           double zoom){
        is_filtered=false;
        this->division_x = division_x;
        this->division_y = division_y;
        this->TRollingShutter = TRollingShutter;
        this->IK = IK;
        this->matIntrinsic = matIntrinsic;
        this->image_width = image_width;
        this->image_height = image_height;
        this->zoom = zoom;

        raw_angle.resize(angle_quaternion.size(),3);

        for(int i=0,e=angle_quaternion.size();i<e;++i){
            auto el = Quaternion2Vector(angle_quaternion[i]);//require Quaternion2Matrix<3,1>()
            raw_angle(i,0) = el[0];
            raw_angle(i,1) = el[1];
            raw_angle(i,2) = el[2];
        }
        is_filtered = false;
    }

    vector<double> getRow(int r);

    const Eigen::MatrixXd &data();

    template <class T> void setFilterCoeff(T coeff){
        filter_coeff.resize(coeff.size(),1);
        for(int i=0,e=coeff.size();i<e;++i){
            filter_coeff(i,0) = coeff[i];
        }
    }

    const Eigen::MatrixXd &filteredData();

    static Eigen::VectorXd getKaiserWindow(uint32_t tap_length, uint32_t alpha);

    static Eigen::VectorXcd getLPFFrequencyCoeff(uint32_t N, uint32_t alpha, double fs, double fc);
    static Eigen::VectorXcd getKaizerWindowFrequencyCoeff(double alpha, int32_t window_length, int32_t data_length);
    static void Angle2CLerpedFrequency(double fs, double fc, const Eigen::MatrixXd &raw_angle, Eigen::MatrixXcd &freq_vectors);
    static void Frequency2Angle(Eigen::MatrixXcd &frequency_vector_, Eigen::MatrixXd &angle_);
    static void MatrixXcd2VectorXd(const Eigen::MatrixXcd &src, Eigen::VectorXd &dst);
    static void VectorXd2MatrixXcd(const Eigen::VectorXd &src, Eigen::MatrixXcd &dst);

    const Eigen::MatrixXd &filteredDataDFT(double fs, double fc);
    const Eigen::MatrixXd &filteredDataDFTTimeDomainOptimize(double fs, double fc, const Eigen::MatrixXd &coeff);
    Eigen::MatrixXd &filteredDataDFT();
    Eigen::Quaternion<double> toRawQuaternion(uint32_t frame);
    Eigen::Quaternion<double> toFilteredQuaternion(uint32_t frame);
    Eigen::Quaternion<double> toDiffQuaternion(uint32_t frame);

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
    template <typename T_num> static Eigen::Vector3d Quaternion2Vector(Eigen::Quaternion<T_num> q){
        double denom = sqrt(1-q.w()*q.w());
        if(denom==0.0){//まったく回転しない時は０割になるので、場合分けする
            return Eigen::Vector3d(0,0,0);//return zero vector
        }
        return Eigen::Vector3d(q.x(),q.y(),q.z())*2.0*atan2(denom,q.w())/denom;
    }

    /**
     * @param シングルローテーションを表すベクトルを回転を表すクォータニオンへ変換
     **/
    template <typename T_num> static Eigen::Quaternion<T_num> Vector2Quaternion(Eigen::Vector3d w){
        double theta = w.norm();//sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]);//回転角度を計算、normと等しい
        //0割を回避するためにマクローリン展開
        if(theta > EPS){
            auto n = w.normalized();//w * (1.0/theta);//単位ベクトルに変換
            //            double sin_theta_2 = sin(theta*0.5);
            //            return Eigen::Quaternion<T_num>(cos(theta*0.5),n[0]*sin_theta_2,n[1]*sin_theta_2,n[2]*sin_theta_2);
            Eigen::VectorXd n_sin_theta_2 = n * sin(theta*0.5);
            return Eigen::Quaternion<T_num>(cos(theta*0.5),n_sin_theta_2[0],n_sin_theta_2[1],n_sin_theta_2[2]);
        }else{
            return Eigen::Quaternion<T_num>(1.0,0.5*w[0],0.5*w[1],0.5*w[2]);
        }
    }

    /**
     * @param 回転を表すクォータニオンをシングルローテーションを表すベクトルへ変換。前回算出したベクトルを引数として受け取ることで、アンラッピングする。
     * */
    template <typename T_num> static Eigen::Vector3d Quaternion2Vector(Eigen::Quaternion<T_num> &q, Eigen::Vector3d &prev){
        double denom = sqrt(1-q.w()*q.w());
        if(denom==0.0){//まったく回転しない時は０割になるので、場合分けする
            return Eigen::Vector3d(0,0,0);//return zero vector
        }
        double theta_2 = atan2(denom,q.w());
        double prev_theta_2 = prev.norm()/2;
        double diff = theta_2 - prev_theta_2;
        theta_2 -= 2.0*M_PI*(double)(static_cast<int>(diff/(2.0*M_PI)));//マイナスの符号に注意
        //~ printf("Theta_2:%4.3f sc:%d\n",theta_2,static_cast<int>(diff/(2.0*M_PI)));
        if(static_cast<int>(diff/(2.0*M_PI))!=0){
            printf("\n###########Unwrapping %d\n",static_cast<int>(diff/(2.0*M_PI)));
        }

        return Eigen::Vector3d(q.x(),q.y(),q.z())*2.0*theta_2/denom;
    }

    /**
     * @param 回転を表すクォータニオンから回転を表す行列を生成
     **/
    template <typename T_num> static void Quaternion2Matrix(Eigen::Quaternion<T_num> &q, Eigen::MatrixXd &det){
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
    template <typename _Tp, typename _Tx> bool getDistortUnrollingContour(
            Eigen::Quaternion<_Tp> &prevAngleQuaternion,
            Eigen::Quaternion<_Tp> &currAngleQuaternion,
            Eigen::Quaternion<_Tp> &nextAngleQuaternion,
//            uint32_t division_x,
//            uint32_t division_y,
//            double TRollingShutter,
//            Eigen::MatrixXd &IK,
//            Eigen::MatrixXd &matIntrinsic,
//            uint32_t image_width,
//            uint32_t image_height,
            std::vector<_Tx> &vecPorigonn_uv
//            double zoom
            ){

        bool retval = true;


        //手順
        //1.補正前画像を分割した時の分割点の座標(pixel)を計算
        //2.1の座標を入力として、各行毎のW(t1,t2)を計算
        //3.補正後の画像上のポリゴン座標(pixel)を計算、歪み補正も含める

        double fx = matIntrinsic(0, 0);
        double fy = matIntrinsic(1, 1);
        double cx = matIntrinsic(0, 2);
        double cy = matIntrinsic(1, 2);
        double k1 = IK(0,0);
        double k2 = IK(0,1);
        double p1 = IK(0,2);
        double p2 = IK(0,3);

        vecPorigonn_uv.clear();

        //top
        //    for(int j=0;j<=division_y;++j){
        {
            int j=0;
            //W(t1,t2)を計算
            Eigen::MatrixXd R;
            //1
            double v = (double)j/division_y*image_height;

            double exposureTimingInEachRow = TRollingShutter*v/image_height;	//ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]

            Eigen::Quaternion<double> slerpedAngleQuaternion;
            if(exposureTimingInEachRow >= 0){
                slerpedAngleQuaternion = currAngleQuaternion.slerp(exposureTimingInEachRow,nextAngleQuaternion);
            }else{
                slerpedAngleQuaternion = prevAngleQuaternion.slerp(1.0+exposureTimingInEachRow,currAngleQuaternion);
            }
            Quaternion2Matrix(slerpedAngleQuaternion,R);
            for(int i=0;i<=division_x;++i){
                double u = (double)i/division_x*image_width;
                //後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
                Eigen::Vector3d p;
                p << (u- cx)/fx, (v - cy)/fy, 1.0;	//1のポリゴン座標に、K^-1を掛けた結果の３x１行列
                //2
                Eigen::MatrixXd XYW = R * p;//inv()なし

                if(XYW(2,0) < 0.0){
                    retval = false;
                }

                double x1 = XYW(0, 0)/XYW(2, 0);
                double y1 = XYW(1, 0)/XYW(2, 0);

                double r = sqrt(x1*x1+y1*y1);

                double x2 = x1*(1.0+k1*r*r+k2*r*r*r*r)+2.0*p1*x1*y1+p2*(r*r+2.0*x1*x1);
                double y2 = y1*(1.0+k1*r*r+k2*r*r*r*r)+p1*(r*r+2.0*y1*y1)+2.0*p2*x1*y1;
                //変な折り返しを防止
                if((pow(x2-x1,2)>1.0)||(pow(y2-y1,2)>1.0)){
                    //                printf("折り返し防止\r\n");
                    x2 = x1;
                    y2 = y1;
                }
                vecPorigonn_uv.push_back(x2*fx*zoom/image_width*2.0);
                vecPorigonn_uv.push_back(y2*fy*zoom/image_height*2.0);
            }
        }

        //middle
        for(int j=1;j<division_y;++j){
            //W(t1,t2)を計算
            Eigen::MatrixXd R;
            //1
            double v = (double)j/division_y*image_height;

            double exposureTimingInEachRow = TRollingShutter*v/image_height;	//ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]

            Eigen::Quaternion<double> slerpedAngleQuaternion;
            if(exposureTimingInEachRow >= 0){
                slerpedAngleQuaternion = currAngleQuaternion.slerp(exposureTimingInEachRow,nextAngleQuaternion);
            }else{
                slerpedAngleQuaternion = prevAngleQuaternion.slerp(1.0+exposureTimingInEachRow,currAngleQuaternion);
            }
            Quaternion2Matrix(slerpedAngleQuaternion,R);
            for(int i=0;i<=division_x;i+=division_x){
                double u = (double)i/division_x*image_width;
                //後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
                Eigen::Vector3d p;
                p << (u- cx)/fx, (v - cy)/fy, 1.0;	//1のポリゴン座標に、K^-1を掛けた結果の３x１行列
                //2
                Eigen::MatrixXd XYW = R * p;//inv()なし

                if(XYW(2,0) < 0.0){
                    retval = false;
                }

                double x1 = XYW(0, 0)/XYW(2, 0);
                double y1 = XYW(1, 0)/XYW(2, 0);

                double r = sqrt(x1*x1+y1*y1);

                double x2 = x1*(1.0+k1*r*r+k2*r*r*r*r)+2.0*p1*x1*y1+p2*(r*r+2.0*x1*x1);
                double y2 = y1*(1.0+k1*r*r+k2*r*r*r*r)+p1*(r*r+2.0*y1*y1)+2.0*p2*x1*y1;
                //変な折り返しを防止
                if((pow(x2-x1,2)>1.0)||(pow(y2-y1,2)>1.0)){
                    //                printf("折り返し防止\r\n");
                    x2 = x1;
                    y2 = y1;
                }
                vecPorigonn_uv.push_back(x2*fx*zoom/image_width*2.0);
                vecPorigonn_uv.push_back(y2*fy*zoom/image_height*2.0);
            }
        }

        //bottom
        {
            int j=division_y;
            //W(t1,t2)を計算
            Eigen::MatrixXd R;
            //1
            double v = (double)j/division_y*image_height;

            double exposureTimingInEachRow = TRollingShutter*v/image_height;	//ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]

            Eigen::Quaternion<double> slerpedAngleQuaternion;
            if(exposureTimingInEachRow >= 0){
                slerpedAngleQuaternion = currAngleQuaternion.slerp(exposureTimingInEachRow,nextAngleQuaternion);
            }else{
                slerpedAngleQuaternion = prevAngleQuaternion.slerp(1.0+exposureTimingInEachRow,currAngleQuaternion);
            }
            Quaternion2Matrix(slerpedAngleQuaternion,R);
            for(int i=0;i<=division_x;++i){
                double u = (double)i/division_x*image_width;
                //後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
                Eigen::Vector3d p;
                p  << (u- cx)/fx, (v - cy)/fy, 1.0;	//1のポリゴン座標に、K^-1を掛けた結果の３x１行列
                //2
                Eigen::MatrixXd XYW = R * p;//inv()なし

                if(XYW(2,0) < 0.0){
                    retval = false;
                }

                double x1 = XYW(0, 0)/XYW(2, 0);
                double y1 = XYW(1, 0)/XYW(2, 0);

                double r = sqrt(x1*x1+y1*y1);

                double x2 = x1*(1.0+k1*r*r+k2*r*r*r*r)+2.0*p1*x1*y1+p2*(r*r+2.0*x1*x1);
                double y2 = y1*(1.0+k1*r*r+k2*r*r*r*r)+p1*(r*r+2.0*y1*y1)+2.0*p2*x1*y1;
                //変な折り返しを防止
                if((pow(x2-x1,2)>1.0)||(pow(y2-y1,2)>1.0)){
                    //                printf("折り返し防止\r\n");
                    x2 = x1;
                    y2 = y1;
                }
                vecPorigonn_uv.push_back(x2*fx*zoom/image_width*2.0);
                vecPorigonn_uv.push_back(y2*fy*zoom/image_height*2.0);
            }
        }

        return retval;
    }

    /**
     * @brief ワープした時に欠けがないかチェックします
     * @retval false:欠けあり true:ワープが良好
     **/
    template <typename _Tp> static bool check_warp(vector<_Tp> &contour){
        for(int i=0;i<contour.size();i+=2){
            if((abs(contour[i]) < 1.0)&&(abs(contour[i+1]) < 1.0)){
                return false;
            }
        }
        return true;
    }

    /**
     * @brief 画面の欠けを生み出している回転ベクトルのオーバーした長さを返す
     **/
    Eigen::VectorXd getRollingVectorError();

private:
    Eigen::MatrixXd raw_angle;
    Eigen::MatrixXd filtered_angle;
    Eigen::VectorXd filter_coeff;
    bool is_filtered;

    int32_t division_x = 9;
    int32_t division_y = 9;
    double TRollingShutter = 0.0;
    Eigen::MatrixXd IK = Eigen::MatrixXd::Zero(1,4);
    Eigen::MatrixXd matIntrinsic = Eigen::MatrixXd::Identity(3,3);
    int32_t image_width=1920;
    int32_t image_height=1080;
    double zoom=1.0;
};

#endif // VSP_H
