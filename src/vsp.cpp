#include "vsp.h"
#include <boost/math/special_functions/bessel.hpp>

vsp::vsp()
{
    is_filterd=false;
}

//template <class T> vsp::vsp(vector<quaternion<T>> &angle_quaternion)
Eigen::Quaternion<double> vsp::RotationQuaternion(double theta, Eigen::Vector3d n){
    //nを規格化してから計算する
    auto n_sin_theta_2 = n.normalized()*sin(theta/2.0);
    return Eigen::Quaternion<double>(cos(theta/2),n_sin_theta_2[0],n_sin_theta_2[1],n_sin_theta_2[2]);
}

/**
 * @brief 微小回転を表す回転ベクトルから四元数を作る関数
 **/

Eigen::Quaternion<double> vsp::RotationQuaternion(Eigen::Vector3d w){
    double theta = w.norm();//!<回転角
    if(theta == 0.0){
        return Eigen::Quaternion<double>(1,0,0,0);
    }
//    auto n = w*(1.0/theta);								//!<回転軸を表す単位ベクトル
    return RotationQuaternion(theta, w.normalized());
}

vector<double> vsp::getRow(int r){
    vector<double> retval;
    for(int i=0,e=raw_angle.rows();i<e;++i){
        retval.push_back(raw_angle(i,r));
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
        int full_tap_length = filter_coeff.rows();
        filtered_angle.resize(raw_angle.rows()-(full_tap_length-1),3);

        for(int i=0,e=filtered_angle.cols();i<e;++i){
            filtered_angle.block(i,0,1,3) = filter_coeff.transpose() * raw_angle.block(i,0,full_tap_length,3);
        }

        is_filterd = true;
        return filtered_angle;
    }
}



Eigen::Quaternion<double> vsp::toRawQuaternion(uint32_t frame){
    int half_tap_length = filter_coeff.rows()/2;
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
    int half_tap_length = filter_coeff.rows()/2;
    Eigen::VectorXd w = filtered_angle.block(frame+half_tap_length,0,1,3);
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

Eigen::VectorXd vsp::getKaiserWindow(uint32_t tap_length, uint32_t alpha){
    Eigen::VectorXd window = Eigen::VectorXd::Zero(tap_length);

    if(tap_length % 2){ //奇数
        int32_t L = tap_length/2;
        for(int32_t n=-L,e=L;n<=e;++n){
            window[n+L] = boost::math::cyl_bessel_i(0.0,alpha*sqrt(1.0-pow((double)n/(double)L,2.0)))
                    /boost::math::cyl_bessel_i(0.0,alpha);
//            std::cout << n+L << std::endl;
        }
    }else{  //偶数
        int32_t L = tap_length/2;
        for(int32_t n=-L,e=L;n<e;++n){//異なる終了条件
            window[n+L] = boost::math::cyl_bessel_i(0.0,alpha*sqrt(1.0-pow((double)n/(double)L,2.0)))
                    /boost::math::cyl_bessel_i(0.0,alpha);
//            std::cout << n+L << std::endl;
        }
    }

    Eigen::VectorXd buff2(window.rows());
    buff2.block(0,0,window.rows()/2,1) = window.block(window.rows()/2,0,window.rows()/2,1);
    buff2.block(window.rows()/2,0,window.rows()-window.rows()/2,1) = window.block(0,0,window.rows()-window.rows()/2,1);

    return buff2;
}

//std::vector<std::complex<double>> vsp::getLPFFrequencyCoeff(uint32_t N, uint32_t alpha, double fs, double fc){
Eigen::VectorXcd vsp::getLPFFrequencyCoeff(uint32_t N, uint32_t alpha, double fs, double fc){
    Eigen::VectorXd time_vector = Eigen::VectorXd::Zero(N);
    Eigen::VectorXcd frequency_vector = Eigen::VectorXcd::Zero(N);
    int32_t Nc = (int32_t)((double)N * fc / fs);
    for(int i=0;i<(Nc-1);++i){
        frequency_vector[i].real(1.0);
        frequency_vector[N-1-i].real(1.0);
    }
    for(int i=(Nc-1);i<(N-Nc);++i){
        frequency_vector[i].real(0.0);
    }
//    for(auto &el:frequency_vector){
//    for(auto &el:frequency_vector){
//        el.imag(0.0);
//    }
    Eigen::FFT<double> fft;
    fft.inv(time_vector,frequency_vector);
    Eigen::VectorXd time_eigen_vector = Eigen::Map<Eigen::VectorXd>(&time_vector[0],time_vector.size());
    time_eigen_vector = time_eigen_vector.array() * getKaiserWindow(N,alpha).array();
    Eigen::Map<Eigen::VectorXd>(&time_vector[0],time_vector.size()) = time_eigen_vector;
    fft.fwd(frequency_vector,time_vector);

    return frequency_vector;
}



const Eigen::MatrixXd &vsp::filteredDataDCT(double fs, double fc){
    //TODO: メンバ変数にraw_angle_2を定義する。これは前後にFIRフィルタの分の余白が付いていないものである。
    //TODO: VectorXcdを返すgetLPTFrequencyCoeffをメンバ関数に定義する
    Eigen::VectorXcd freq_vector;
    Eigen::FFT<double> fft;
//    Eigen::VectorXd raw_angle_x = raw_angle.block(0,0,1,raw_angle.cols()).transpose();
//    Eigen::VectorXd raw_angle_y = raw_angle.block(1,0,1,raw_angle.cols()).transpose();
//    Eigen::VectorXd raw_angle_z = raw_angle.block(2,0,1,raw_angle.cols()).transpose();
    //TODO:後ろにスムーズに接続するための余白をraw_angle_2の末尾に追加

    Eigen::VectorXcd filter_coeff_vector = getLPFFrequencyCoeff(raw_angle.rows(),8,fs,fc).array();
    filtered_angle.resize(raw_angle.rows(),raw_angle.cols());

    //Apply LPF to each x, y and z axis.
    for(int i=0,e=raw_angle.cols();i<e;++i){
        freq_vector = fft.fwd(raw_angle.col(i));
        freq_vector = filter_coeff_vector.array() * freq_vector.array();
        filtered_angle.col(i).noalias() = fft.inv(freq_vector);
    }
    //TODO: ここで末尾の余白を削除
}

Eigen::MatrixXd vsp::CLerp(Eigen::MatrixXd start, Eigen::MatrixXd end, int32_t num){
    assert(start.cols() == end.cols());
    assert(start.rows() == end.rows());
    Eigen::VectorXd phase = Eigen::ArrayXd::LinSpaced(num,0,M_PI);
    Eigen::VectorXd cos_phase = phase.array().cos();
    return cos_phase * (-(end - start)*0.5) + Eigen::VectorXd::Ones(phase.rows()) * (start + end) * 0.5;
}
