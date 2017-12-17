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

const Eigen::MatrixXd vsp::data(){
    return raw_angle;
}

const Eigen::MatrixXd &vsp::filterdData(){
    if(is_filterd){
        return filterd_angle;
    }else{
        int full_tap_length = filter_coeff.cols();
        filterd_angle.resize(3,raw_angle.cols()-full_tap_length-1);

        for(int i=0,e=filterd_angle.cols();i<e;++i){
            filterd_angle.block(0,i,3,1) = raw_angle.block(0,i,3,full_tap_length)*filter_coeff.transpose();
        }

        is_filterd = true;
        return filterd_angle;
    }
}
