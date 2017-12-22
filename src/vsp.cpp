#include "vsp.h"

vsp::vsp()
{
    is_filterd=false;
}

//template <class T> vsp::vsp(vector<quaternion<T>> &angle_quaternion)
Eigen::Quaternion<double> vsp::RotationQuaternion(double theta, Eigen::Vector3d n){
    //nを規格化する
//    double tmp = 1.0/sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);
//    n = n * tmp;
    auto n_sin_theta_2 = n.normalized()*sin(theta/2.0);
//    return Eigen::Quaternion<double>(cos(theta/2),n[0]*sin(theta/2),n[1]*sin(theta/2),n[2]*sin(theta/2));
    return Eigen::Quaternion<double>(cos(theta/2),n_sin_theta_2[0],n_sin_theta_2[1],n_sin_theta_2[2]);
}

/**
 * @brief 微小回転を表す回転ベクトルから四元数を作る関数
 **/

Eigen::Quaternion<double> vsp::RotationQuaternion(Eigen::Vector3d w){
    double theta = w.norm();//sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]);	//!<回転角
    if(theta == 0.0){
        return Eigen::Quaternion<double>(1,0,0,0);
    }
//    auto n = w*(1.0/theta);								//!<回転軸を表す単位ベクトル
    return RotationQuaternion(theta, w.normalized());
}

vector<double> vsp::getRow(int r){
    vector<double> retval;
    for(int i=0,e=raw_angle.cols();i<e;++i){
        retval.push_back(raw_angle(r,i));
    }
    return retval;
}

const Eigen::MatrixXd &vsp::data(){
    return raw_angle;
}

const Eigen::MatrixXd &vsp::filteredData(){
    if(is_filterd){
        return filtered_angle;
    }else{
        int full_tap_length = filter_coeff.cols();
        filtered_angle.resize(3,raw_angle.cols()-(full_tap_length-1));

        for(int i=0,e=filtered_angle.cols();i<e;++i){
            filtered_angle.block(0,i,3,1) = raw_angle.block(0,i,3,full_tap_length)*filter_coeff.transpose();
        }

        is_filterd = true;
        return filtered_angle;
    }
}

Eigen::Quaternion<double> vsp::toRawQuaternion(uint32_t frame){
    int half_tap_length = filter_coeff.cols()/2;
    Eigen::VectorXd w = raw_angle.block(0,frame+half_tap_length,3,1);
    double theta = w.norm();//回転角度を計算、normと等しい
    //0割を回避するためにマクローリン展開
    //Order: w, x, y, z.
    if(theta > EPS){
        Eigen::VectorXd n = w.normalized();//w * (1.0/theta);//単位ベクトルに変換
//        double sin_theta_2 = sin(theta*0.5);
        Eigen::VectorXd n_sin_theta_2 = n * sin(theta*0.5);
        return Eigen::Quaternion<double>(cos(theta*0.5),n_sin_theta_2[0],n_sin_theta_2[1],n_sin_theta_2[2]);
    }else{
        return Eigen::Quaternion<double>(1.0,0.5*w[0],0.5*w[1],0.5*w[2]);
    }
}

Eigen::Quaternion<double> vsp::toFilteredQuaternion(uint32_t frame){
    int half_tap_length = filter_coeff.cols()/2;
    Eigen::VectorXd w = filtered_angle.block(0,frame+half_tap_length,3,1);
    double theta = w.norm();//回転角度を計算
    //0割を回避するためにマクローリン展開
    //Order: w, x, y, z.
    if(theta > EPS){
        Eigen::VectorXd n = w.normalized();//w * (1.0/theta);//単位ベクトルに変換
//        double sin_theta_2 = sin(theta*0.5);
        Eigen::VectorXd n_sin_theta_2 = n * sin(theta*0.5);
        return Eigen::Quaternion<double>(cos(theta*0.5),n_sin_theta_2[0],n_sin_theta_2[1],n_sin_theta_2[2]);
    }else{
        return Eigen::Quaternion<double>(1.0,0.5*w[0],0.5*w[1],0.5*w[2]);
    }
}

Eigen::Quaternion<double> vsp::toDiffQuaternion(uint32_t frame){
    return this->toFilteredQuaternion(frame).conjugate()*this->toRawQuaternion(frame);
}
