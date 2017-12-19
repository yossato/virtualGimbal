#ifndef VSP_H
#define VSP_H

#include <stdio.h>
#include <boost/math/quaternion.hpp>
#include "settings.h"
#include <Eigen/Dense>

using namespace std;
using namespace boost::math;

class vsp
{
public:
    vsp();

    //TODO:コンストラクタでfilter coeffも受け取っといたほうがよさ気
    template <class T> vsp(vector<Eigen::Quaternion<T>> &angle_quaternion){
        raw_angle.resize(3,angle_quaternion.size());

        for(int i=0,e=angle_quaternion.size();i<e;++i){
            auto el = Quaternion2Vector(angle_quaternion[i]);//require Quaternion2Matrix<3,1>()
            raw_angle(0,i) = el[0];
            raw_angle(1,i) = el[1];
            raw_angle(2,i) = el[2];
        }
        is_filterd = false;
    }

    vector<double> getRow(int r);

    const Eigen::MatrixXd &data();

    template <class T> void setFilterCoeff(T coeff){
        filter_coeff.resize(1,coeff.size());
        for(int i=0,e=coeff.size();i<e;++i){
            filter_coeff(0,i) = coeff[i];
        }
    }

    const Eigen::MatrixXd &filteredData();

    Eigen::Quaternion<double> toRawQuaternion(uint32_t frame);
    Eigen::Quaternion<double> toFilteredQuaternion(uint32_t frame);
    Eigen::Quaternion<double> toDiffQuaternion(uint32_t frame);

    /**
     * @param 回転を表すクォータニオンをシングルローテーションをあらわすベクトルへ変換
     **/
    template <typename T_num> Eigen::Vector3d Quaternion2Vector(Eigen::Quaternion<T_num> q){
        double denom = sqrt(1-q.R_component_1()*q.R_component_1());
        if(denom==0.0){//まったく回転しない時は０割になるので、場合分けする
            return Eigen::Vector3d(0,0,0);//return zero vector
        }
        return Eigen::Vector3d(q.R_component_2(),q.R_component_3(),q.R_component_4())*2.0*atan2(denom,q.R_component_1())/denom;
    }

    /**
     * @param シングルローテーションを表すベクトルを回転を表すクォータニオンへ変換
     **/
    template <typename T_num> Eigen::Quaternion<T_num> Vector2Quaternion(Eigen::Vector3d w){
        double theta = sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]);//回転角度を計算、normと等しい
        //0割を回避するためにマクローリン展開
        if(theta > EPS){
            auto n = w * (1.0/theta);//単位ベクトルに変換
            double sin_theta_2 = sin(theta*0.5);
            return Eigen::Quaternion<double>(cos(theta*0.5),n[0]*sin_theta_2,n[1]*sin_theta_2,n[2]*sin_theta_2);
        }else{
            return Eigen::Quaternion<double>(1.0,0.5*w[0],0.5*w[1],0.5*w[2]);
        }
    }

    /**
     * @param 回転を表すクォータニオンをシングルローテーションを表すベクトルへ変換。前回算出したベクトルを引数として受け取ることで、アンラッピングする。
     * */
    template <typename T_num> Eigen::Vector3d Quaternion2Vector(Eigen::Quaternion<T_num> q, Eigen::Vector3d prev){
        double denom = sqrt(1-q.R_component_1()*q.R_component_1());
        if(denom==0.0){//まったく回転しない時は０割になるので、場合分けする
            return Eigen::Vector3d(0,0,0);//return zero vector
        }
        double theta_2 = atan2(denom,q.R_component_1());
        double prev_theta_2 = prev.norm()/2;
        double diff = theta_2 - prev_theta_2;
        theta_2 -= 2.0*M_PI*(double)(static_cast<int>(diff/(2.0*M_PI)));//マイナスの符号に注意
        //~ printf("Theta_2:%4.3f sc:%d\n",theta_2,static_cast<int>(diff/(2.0*M_PI)));
        if(static_cast<int>(diff/(2.0*M_PI))!=0){
            printf("\n###########Unwrapping %d\n",static_cast<int>(diff/(2.0*M_PI)));
        }

        return Eigen::Vector3d(q.R_component_2(),q.R_component_3(),q.R_component_4())*2.0*theta_2/denom;
    }

    /**
     * @param 回転を表すクォータニオンから回転を表す行列を生成
     **/
    template <typename T_num> void Quaternion2Matrix(Eigen::Quaternion<T_num> q, Eigen::MatrixXd &det){
        det = (Eigen::MatrixXd(3,3) <<
               q.R_component_1()*q.R_component_1()+q.R_component_2()*q.R_component_2()-q.R_component_3()*q.R_component_3()-q.R_component_4()*q.R_component_4(), 2*(q.R_component_2()*q.R_component_3()-q.R_component_1()*q.R_component_4()),                  2*(q.R_component_2()*q.R_component_4()+q.R_component_1()*q.R_component_3()),
               2*(q.R_component_2()*q.R_component_3()+q.R_component_1()*q.R_component_4()),                 q.R_component_1()*q.R_component_1()-q.R_component_2()*q.R_component_2()+q.R_component_3()*q.R_component_3()-q.R_component_4()*q.R_component_4(), 2*(q.R_component_3()*q.R_component_4()-q.R_component_1()*q.R_component_2()),
               2*(q.R_component_2()*q.R_component_4()-q.R_component_1()*q.R_component_3()),                 2*(q.R_component_3()*q.R_component_4()+q.R_component_1()*q.R_component_2()),                  q.R_component_1()*q.R_component_1()-q.R_component_2()*q.R_component_2()-q.R_component_3()*q.R_component_3()+q.R_component_4()*q.R_component_4()
               );
    }

    /**
     * @brief 球面線形補間関数
     * @param [in]	Qfrom	四元数1
     * @param [in]	Qto		四元数2
     * @param [in]	t		比率(0<=t<=1)
     **/
    template <typename _Tp> Eigen::Quaternion<_Tp> Slerp(Eigen::Quaternion<_Tp> Qfrom, Eigen::Quaternion<_Tp> Qto, _Tp t){
        double cosom = Qfrom.R_component_1()*Qto.R_component_1()+Qfrom.R_component_2()*Qto.R_component_2()+Qfrom.R_component_3()*Qto.R_component_3()+Qfrom.R_component_4()*Qto.R_component_4();
        double sinom, omega, scale0, scale1;

        if(Qto == Qfrom){	//QfromとQtoが完全に一致->補完の必要なし
            return Qfrom;
        }

        //符号を直す
        if(cosom < 0.0){
            cosom = -cosom;
            Qto = -Qto;
        }
        if((1.0-cosom)>1e-4){
            omega = acos(cosom);
            sinom = sin(omega);
            scale0 = sin((1.0 - t) * omega) / sinom;
            scale1 = sin(t * omega) / sinom;
        }else{
            scale0 = 1.0 -t;
            scale1 = t;
        }

        return scale0 * Qfrom + scale1 * Qto;

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
            uint32_t division_x,
            uint32_t division_y,
            double TRollingShutter,
            Eigen::MatrixXd &IK,
            Eigen::MatrixXd &matIntrinsic,
            cv::Size imageSize,
            Eigen::Quaternion<double> adjustmentQuaternion,
            std::vector<_Tx> &vecPorigonn_uv,
            double zoom
            ){

        bool retval = true;

        //Matの型をdoubleに強制。
//        assert(IK.type() == CV_64F);
//        assert(matIntrinsic.type() == CV_64F);

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

        Eigen::MatrixXd map(division_y+1,division_x+1,CV_64FC2);

        vecPorigonn_uv.clear();

        //top
    //    for(int j=0;j<=division_y;++j){
        {
            int j=0;
            //W(t1,t2)を計算
            Eigen::MatrixXd R;
            //1
            double v = (double)j/division_y*imageSize.height;

            double exposureTimingInEachRow = TRollingShutter*v/imageSize.height;	//ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]

            Eigen::Quaternion<double> slerpedAngleQuaternion;
            if(exposureTimingInEachRow >= 0){
                slerpedAngleQuaternion = Slerp(currAngleQuaternion,nextAngleQuaternion,exposureTimingInEachRow);
            }else{
                slerpedAngleQuaternion = Slerp(prevAngleQuaternion,currAngleQuaternion,1+exposureTimingInEachRow);
            }
            slerpedAngleQuaternion = adjustmentQuaternion * slerpedAngleQuaternion;
            Quaternion2Matrix(slerpedAngleQuaternion,R);
            for(int i=0;i<=division_x;++i){
                double u = (double)i/division_x*imageSize.width;
                //後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
                Eigen::MatrixXd p = (Eigen::MatrixXd(3,1) << (u- cx)/fx, (v - cy)/fy, 1.0);	//1のポリゴン座標に、K^-1を掛けた結果の３x１行列
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
                vecPorigonn_uv.push_back(x2*fx*zoom/imageSize.width*2.0);
                vecPorigonn_uv.push_back(y2*fy*zoom/imageSize.height*2.0);
    //            vecPorigonn_uv.push_back(x1*fx*zoom/imageSize.width*2.0);
    //                        vecPorigonn_uv.push_back(y1*fy*zoom/imageSize.height*2.0);
            }
        }

        //middle
        for(int j=1;j<division_y;++j){
            //W(t1,t2)を計算
            Eigen::MatrixXd R;
            //1
            double v = (double)j/division_y*imageSize.height;

            double exposureTimingInEachRow = TRollingShutter*v/imageSize.height;	//ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]

            Eigen::Quaternion<double> slerpedAngleQuaternion;
            if(exposureTimingInEachRow >= 0){
                slerpedAngleQuaternion = Slerp(currAngleQuaternion,nextAngleQuaternion,exposureTimingInEachRow);
            }else{
                slerpedAngleQuaternion = Slerp(prevAngleQuaternion,currAngleQuaternion,1+exposureTimingInEachRow);
            }
            slerpedAngleQuaternion = adjustmentQuaternion * slerpedAngleQuaternion;
            Quaternion2Matrix(slerpedAngleQuaternion,R);
            for(int i=0;i<=division_x;i+=division_x){
                double u = (double)i/division_x*imageSize.width;
                //後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
                Eigen::MatrixXd p = (Eigen::MatrixXd(3,1) << (u- cx)/fx, (v - cy)/fy, 1.0);	//1のポリゴン座標に、K^-1を掛けた結果の３x１行列
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
                vecPorigonn_uv.push_back(x2*fx*zoom/imageSize.width*2.0);
                vecPorigonn_uv.push_back(y2*fy*zoom/imageSize.height*2.0);
            }
        }

        //bottom
        {
            int j=division_y;
            //W(t1,t2)を計算
            Eigen::MatrixXd R;
            //1
            double v = (double)j/division_y*imageSize.height;

            double exposureTimingInEachRow = TRollingShutter*v/imageSize.height;	//ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]

            Eigen::Quaternion<double> slerpedAngleQuaternion;
            if(exposureTimingInEachRow >= 0){
                slerpedAngleQuaternion = Slerp(currAngleQuaternion,nextAngleQuaternion,exposureTimingInEachRow);
            }else{
                slerpedAngleQuaternion = Slerp(prevAngleQuaternion,currAngleQuaternion,1+exposureTimingInEachRow);
            }
            slerpedAngleQuaternion = adjustmentQuaternion * slerpedAngleQuaternion;
            Quaternion2Matrix(slerpedAngleQuaternion,R);
            for(int i=0;i<=division_x;++i){
                double u = (double)i/division_x*imageSize.width;
                //後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
                Eigen::MatrixXd p = (Eigen::MatrixXd(3,1) << (u- cx)/fx, (v - cy)/fy, 1.0);	//1のポリゴン座標に、K^-1を掛けた結果の３x１行列
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
                vecPorigonn_uv.push_back(x2*fx*zoom/imageSize.width*2.0);
                vecPorigonn_uv.push_back(y2*fy*zoom/imageSize.height*2.0);
            }
        }

        return retval;
    }

    /**
     * @brief ワープした時に欠けがないかチェックします
     * @retval false:欠けあり true:ワープが良好
     **/
    template <typename _Tp> bool check_warp(vector<_Tp> &contour){
        for(int i=0;i<contour.size();i+=2){
            if((abs(contour[i]) < 1.0)&&(abs(contour[i+1]) < 1.0)){
                return false;
            }
        }
        return true;
    }

    /**
     * @brief 画面の欠けを生み出している回転ベクトルのオーバーした長さを返す
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
    template <typename _Tp> void getRollingVectorError(
            Eigen::Quaternion<double> &prevAngleQuaternion,
            Eigen::Quaternion<double> &currAngleQuaternion,
            Eigen::Quaternion<double> &nextAngleQuaternion,
            uint32_t division_x,
            uint32_t division_y,
            double TRollingShutter,
            Eigen::MatrixXd &IK,
            Eigen::MatrixXd &matIntrinsic,
            cv::Size imageSize,
            Eigen::Quaternion<double> adjustmentQuaternion,
            double zoom,
            double &error
            ){
        double a=0.0;
        double b=1.0;
        double eps = 1.0e-5;
        int count=0;
        std::vector<float> vecPorigonn_uv;
        auto func = [&](double ratio){
            getDistortUnrollingContour(prevAngleQuaternion,
                                       currAngleQuaternion,
                                       nextAngleQuaternion,
                                       division_x,
                                       division_y,
                                       TRollingShutter,
                                       IK,
                                       matIntrinsic,
                                       imageSize,
                                       Vector2Quaternion<double>(ratio*Quaternion2Vector(adjustmentQuaternion)),
                                       vecPorigonn_uv,
                                       zoom
                                       );
            return check_warp(vecPorigonn_uv);
        };
        if(func(1.0)==true){
            error = 0.0;
            return;
        }
    double m;
        do{
            count++;
            m=(a+b)/2.0;
            if(func(m)^func(a)){
                b=m;
            }else{
                a=m;
            }
            if(count == 1000){
                cout << "収束失敗" << endl;
               break;
            }
        }while(!(abs(a-b)<eps));
        cout << count << "回で収束" << endl;

        error = (Quaternion2Vector(adjustmentQuaternion).norm())*(1-m);
    }

//    const Eigen::Quaternion

private:
    Eigen::MatrixXd raw_angle;
    Eigen::MatrixXd filtered_angle;
    Eigen::MatrixXd filter_coeff;
    bool is_filterd;
};

#endif // VSP_H
