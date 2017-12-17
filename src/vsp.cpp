#include "vsp.h"

vsp::vsp()
{
    is_filterd=false;
}

//template <class T> vsp::vsp(vector<quaternion<T>> &angle_quaternion)

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
        auto n = w * (1.0/theta);//単位ベクトルに変換
//        double sin_theta_2 = sin(theta*0.5);
        auto n_sin_theta_2 = n * sin(theta*0.5);
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
        auto n = w * (1.0/theta);//単位ベクトルに変換
//        double sin_theta_2 = sin(theta*0.5);
        auto n_sin_theta_2 = n * sin(theta*0.5);
        return Eigen::Quaternion<double>(cos(theta*0.5),n_sin_theta_2[0],n_sin_theta_2[1],n_sin_theta_2[2]);
    }else{
        return Eigen::Quaternion<double>(1.0,0.5*w[0],0.5*w[1],0.5*w[2]);
    }
}

Eigen::Quaternion<double> vsp::toDiffQuaternion(uint32_t frame){
    return this->toFilteredQuaternion(frame).conjugate()*this->toRawQuaternion(frame);
}
