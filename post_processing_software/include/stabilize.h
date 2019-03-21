#ifndef STABILIZE_H
#define STABILIZE_H


#include <stdio.h>
#include <boost/math/quaternion.hpp>
#include "settings.h"
using namespace std;
using namespace boost::math;

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
    if(theta > EPS_Q){
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

/**
 * @param 回転を表すクォータニオンから回転を表す行列を生成
 **/
template <typename T_num> void Quaternion2Matrix(quaternion<T_num> q, cv::Mat &det){
    det = (cv::Mat_<T_num>(3,3) <<
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
template <typename _Tp> quaternion<_Tp> Slerp(quaternion<_Tp> Qfrom, quaternion<_Tp> Qto, _Tp t){
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

/** @brief 補正前の画像座標から、補正後のポリゴンの頂点の位置を表す座標の組を作成
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
 **/
template <typename _Tp, typename _Tx> void getDistortUnrollingMap(
        quaternion<_Tp> &prevAngleQuaternion,
        quaternion<_Tp> &currAngleQuaternion,
        quaternion<_Tp> &nextAngleQuaternion,
        uint32_t division_x,
        uint32_t division_y,
        double TRollingShutter,
        cv::Mat &IK,
        cv::Mat &matIntrinsic,
        cv::Size imageSize,
        quaternion<_Tp> adjustmentQuaternion,
        std::vector<_Tx> &vecPorigonn_uv,
        double zoom
        ){

    //Matの型をdoubleに強制。
    assert(IK.type() == CV_64F);
    assert(matIntrinsic.type() == CV_64F);

    //手順
    //1.補正前画像を分割した時の分割点の座標(pixel)を計算
    //2.1の座標を入力として、各行毎のW(t1,t2)を計算
    //3.補正後の画像上のポリゴン座標(pixel)を計算、歪み補正も含める

    double fx = matIntrinsic.at<double>(0, 0);
    double fy = matIntrinsic.at<double>(1, 1);
    double cx = matIntrinsic.at<double>(0, 2);
    double cy = matIntrinsic.at<double>(1, 2);
    double k1 = IK.at<double>(0,0);
    double k2 = IK.at<double>(0,1);
    double p1 = IK.at<double>(0,2);
    double p2 = IK.at<double>(0,3);

    cv::Mat map(division_y+1,division_x+1,CV_64FC2);


    for(int j=0;j<=division_y;++j){
        //W(t1,t2)を計算
        cv::Mat R;
        //1
        double v = (double)j/division_y*imageSize.height;

        double exposureTimingInEachRow = TRollingShutter*v/imageSize.height;	//ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]

        quaternion<_Tp> slerpedAngleQuaternion;
        if(exposureTimingInEachRow >= 0){
            slerpedAngleQuaternion = Slerp(currAngleQuaternion,nextAngleQuaternion,exposureTimingInEachRow);
        }else{
            slerpedAngleQuaternion = Slerp(prevAngleQuaternion,currAngleQuaternion,1+exposureTimingInEachRow);
        }

        //        unsigned int fi = (int)floor(exposureTimingInEachRow/T);	//ローリングシャッター補正を含むフレーム数の整数部[ ]
        //        double ff = exposureTimingInEachRow/T - (double)fi;			//ローリングシャッター補正を含むフレーム数の浮動小数点数部[ ]
        //        auto SQa = Slerp(Qa[fi],Qa[fi+1],ff);	//オリジナルの角度クウォータニオンに関して球面線形補間

        //        unsigned int gi = (int)floor(ti/T);		//フレーム数の整数部[ ]
        //        double gf = ti/T - (double)gi;			//フレーム数の浮動小数点数部[ ]
        //        auto SQf = Slerp(Qf[gi],Qf[gi+1],gf);	//フィルタ済みの角度クウォータニオンに関して球面線形補間

        //        Quaternion2Matrix(conj(SQf)*SQa,R);		//ローリングシャッター補正を含む回転行列を計算

        slerpedAngleQuaternion = adjustmentQuaternion * slerpedAngleQuaternion;

        Quaternion2Matrix(slerpedAngleQuaternion,R);

        for(int i=0;i<=division_x;++i){
            double u = (double)i/division_x*imageSize.width;
            //後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
            cv::Mat p = (cv::Mat_<double>(3,1) << (u- cx)/fx, (v - cy)/fy, 1.0);	//1のポリゴン座標に、K^-1を掛けた結果の３x１行列
            //2
            cv::Mat XYW = R * p;//inv()なし

            double x1 = XYW.at<double>(0, 0)/XYW.at<double>(2, 0);
            double y1 = XYW.at<double>(1, 0)/XYW.at<double>(2, 0);

            double r = sqrt(x1*x1+y1*y1);

            double x2 = x1*(1.0+k1*r*r+k2*r*r*r*r)+2.0*p1*x1*y1+p2*(r*r+2.0*x1*x1);
            double y2 = y1*(1.0+k1*r*r+k2*r*r*r*r)+p1*(r*r+2.0*y1*y1)+2.0*p2*x1*y1;
            //変な折り返しを防止
            if((pow(x2-x1,2)>1.0)||(pow(y2-y1,2)>1.0)){
                //                printf("折り返し防止\r\n");
                x2 = x1;
                y2 = y1;
            }
            //            double mapx = x2*fx*zoom+cx;
            //            double mapy = y2*fy*zoom+cy;
            //~ return ;
            //結果をmapに保存
            //            map.at<cv::Vec2d>(j,i)[0] = mapx/textureSize.width;
            //            map.at<cv::Vec2d>(j,i)[1] = mapy/textureSize.height;
            //            map.at<cv::Vec2d>(j,i)[0] = (mapx-cx)/imageSize.width*2.0;
            //            map.at<cv::Vec2d>(j,i)[1] = (mapy-cy)/imageSize.height*2.0;
            map.at<cv::Vec2d>(j,i)[0] = x2*fx*zoom/imageSize.width*2.0;
            map.at<cv::Vec2d>(j,i)[1] = y2*fy*zoom/imageSize.height*2.0;
            //~ printf("i:%d,j:%d,mapx:%4.3f,mapy:%4.3f\n",i,j,mapx,mapy);
        }
    }

    //    cout << map << "\r\n" << endl;

    //3.ポリゴン座標をOpenGLの関数に渡すために順番を書き換える
    vecPorigonn_uv.clear();
    for(int j=0;j<division_y;++j){//jは終了の判定が"<"であることに注意
        for(int i=0;i<division_x;++i){
            //GL_TRIANGLESでGL側へ送信するポリゴンの頂点座標を準備
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j,i)[0]);//x座標
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j,i)[1]);//y座標
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j,i+1)[0]);//x座標
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j,i+1)[1]);//y座標
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j+1,i)[0]);//x座標
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j+1,i)[1]);//y座標

            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j+1,i)[0]);//x座標
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j+1,i)[1]);//y座標
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j,i+1)[0]);//x座標
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j,i+1)[1]);//y座標
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j+1,i+1)[0]);//x座標
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j+1,i+1)[1]);//y座標


        }
    }




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
        quaternion<_Tp> &prevAngleQuaternion,
        quaternion<_Tp> &currAngleQuaternion,
        quaternion<_Tp> &nextAngleQuaternion,
        uint32_t division_x,
        uint32_t division_y,
        double TRollingShutter,
        cv::Mat &IK,
        cv::Mat &matIntrinsic,
        cv::Size imageSize,
        quaternion<_Tp> adjustmentQuaternion,
        std::vector<_Tx> &vecPorigonn_uv,
        double zoom
        ){

    bool retval = true;

    //Matの型をdoubleに強制。
    assert(IK.type() == CV_64F);
    assert(matIntrinsic.type() == CV_64F);

    //手順
    //1.補正前画像を分割した時の分割点の座標(pixel)を計算
    //2.1の座標を入力として、各行毎のW(t1,t2)を計算
    //3.補正後の画像上のポリゴン座標(pixel)を計算、歪み補正も含める

    double fx = matIntrinsic.at<double>(0, 0);
    double fy = matIntrinsic.at<double>(1, 1);
    double cx = matIntrinsic.at<double>(0, 2);
    double cy = matIntrinsic.at<double>(1, 2);
    double k1 = IK.at<double>(0,0);
    double k2 = IK.at<double>(0,1);
    double p1 = IK.at<double>(0,2);
    double p2 = IK.at<double>(0,3);

    cv::Mat map(division_y+1,division_x+1,CV_64FC2);

    vecPorigonn_uv.clear();

    //top
//    for(int j=0;j<=division_y;++j){
    {
        int j=0;
        //W(t1,t2)を計算
        cv::Mat R;
        //1
        double v = (double)j/division_y*imageSize.height;

        double exposureTimingInEachRow = TRollingShutter*v/imageSize.height;	//ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]

        quaternion<_Tp> slerpedAngleQuaternion;
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
            cv::Mat p = (cv::Mat_<double>(3,1) << (u- cx)/fx, (v - cy)/fy, 1.0);	//1のポリゴン座標に、K^-1を掛けた結果の３x１行列
            //2
            cv::Mat XYW = R * p;//inv()なし

            if(XYW.at<double>(2,0) < 0.0){
                retval = false;
            }

            double x1 = XYW.at<double>(0, 0)/XYW.at<double>(2, 0);
            double y1 = XYW.at<double>(1, 0)/XYW.at<double>(2, 0);

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
        cv::Mat R;
        //1
        double v = (double)j/division_y*imageSize.height;

        double exposureTimingInEachRow = TRollingShutter*v/imageSize.height;	//ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]

        quaternion<_Tp> slerpedAngleQuaternion;
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
            cv::Mat p = (cv::Mat_<double>(3,1) << (u- cx)/fx, (v - cy)/fy, 1.0);	//1のポリゴン座標に、K^-1を掛けた結果の３x１行列
            //2
            cv::Mat XYW = R * p;//inv()なし

            if(XYW.at<double>(2,0) < 0.0){
                retval = false;
            }

            double x1 = XYW.at<double>(0, 0)/XYW.at<double>(2, 0);
            double y1 = XYW.at<double>(1, 0)/XYW.at<double>(2, 0);

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
        cv::Mat R;
        //1
        double v = (double)j/division_y*imageSize.height;

        double exposureTimingInEachRow = TRollingShutter*v/imageSize.height;	//ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]

        quaternion<_Tp> slerpedAngleQuaternion;
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
            cv::Mat p = (cv::Mat_<double>(3,1) << (u- cx)/fx, (v - cy)/fy, 1.0);	//1のポリゴン座標に、K^-1を掛けた結果の３x１行列
            //2
            cv::Mat XYW = R * p;//inv()なし

            if(XYW.at<double>(2,0) < 0.0){
                retval = false;
            }

            double x1 = XYW.at<double>(0, 0)/XYW.at<double>(2, 0);
            double y1 = XYW.at<double>(1, 0)/XYW.at<double>(2, 0);

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
        quaternion<_Tp> &prevAngleQuaternion,
        quaternion<_Tp> &currAngleQuaternion,
        quaternion<_Tp> &nextAngleQuaternion,
        uint32_t division_x,
        uint32_t division_y,
        double TRollingShutter,
        cv::Mat &IK,
        cv::Mat &matIntrinsic,
        cv::Size imageSize,
        quaternion<_Tp> adjustmentQuaternion,
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

    error = cv::norm(Quaternion2Vector(adjustmentQuaternion))*(1-m);
}


#endif // STABILIZE_H
