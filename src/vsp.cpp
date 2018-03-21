#include "vsp.h"
#include <boost/math/special_functions/bessel.hpp>

vsp::vsp(/*vector<Eigen::Quaternion<T>> &angle_quaternion,*/
                       int32_t division_x,
                       int32_t division_y,
                       double TRollingShutter,
                       Eigen::MatrixXd IK,
                       Eigen::MatrixXd matIntrinsic,
                       int32_t image_width,
                       int32_t image_height,
                       double zoom,
                       std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> &angular_velocity,
                       double T_video,
                       double T_angular_velocity,
                       double frame_offset,
                       int32_t video_frames,
                       int32_t filter_tap_length){
    is_filtered=false;
    this->division_x = division_x;
    this->division_y = division_y;
    this->TRollingShutter = TRollingShutter;
    this->IK = IK;
    this->matIntrinsic = matIntrinsic;
    this->image_width = image_width;
    this->image_height = image_height;
    this->zoom = zoom;

    this->angular_velocity = angular_velocity;
    this->T_video = T_video;
    this->T_angular_velocity = T_angular_velocity;
    this->frame_offset = frame_offset;
    this->video_frames = video_frames;
    this->filter_tap_length = filter_tap_length;

    //クォータニオンをクラスの内部で計算する TODO:互換性のためのルーチンなので、後で削除する
    raw_quaternion_with_margin.clear();
    raw_quaternion_with_margin.push_back(Eigen::Quaterniond(1,0,0,0));
    for(int frame= -floor(filter_tap_length/2)-1 ,e=video_frames+floor(filter_tap_length/2)+1;frame<e;++frame){//球面線形補間を考慮し前後各1フレーム追加
        auto v_sync = angularVelocitySync(frame);
        cout << "frame:" << frame << " v_sync:" << v_sync.transpose() << endl;
        raw_quaternion_with_margin.push_back((raw_quaternion_with_margin.back()*vsp::RotationQuaternion(v_sync*this->T_video)).normalized());
    }


    raw_angle.resize(video_frames+2,3);//+2はローリングシャッター歪み補正のため
int32_t half_tap_length = filter_tap_length/2;
    Eigen::Vector3d el = Quaternion2Vector(raw_quaternion_with_margin[0].conjugate());
    for(int i=0,e=raw_angle.rows();i<e;++i){
        el = Quaternion2Vector(raw_quaternion_with_margin[i+half_tap_length].conjugate(),el);//require Quaternion2Matrix<3,1>()
        raw_angle(i,0) = el[0];
        raw_angle(i,1) = el[1];
        raw_angle(i,2) = el[2];
    }

    raw_quaternion.resize(video_frames+2,4);
    for(int i=0,e=raw_quaternion.rows();i<e;++i){
        raw_quaternion.row(i)=raw_quaternion_with_margin[i+half_tap_length].coeffs().transpose();
    }

    is_filtered = false;
}

Eigen::Quaternion<double> vsp::RotationQuaternion(double theta, Eigen::Vector3d n){
    //nを規格化してから計算する
    auto n_sin_theta_2 = n.normalized()*sin(theta/2.0);
    return Eigen::Quaternion<double>(cos(theta/2.0),n_sin_theta_2[0],n_sin_theta_2[1],n_sin_theta_2[2]);
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

const Eigen::MatrixXd &vsp::toQuaternion(){
    return raw_quaternion;
}

//const std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond>> &vsp::toQuaternion_vec(){
//    //クォータニオンをクラスの内部で計算する
//    raw_quaternion_with_margin.clear();
//    raw_quaternion_with_margin.push_back(Eigen::Quaterniond(1,0,0,0));
//    for(int frame= -floor(filter_tap_length/2)-1 ,e=video_frames+floor(filter_tap_length/2)+1;frame<e;++frame){//球面線形補間を考慮し前後各1フレーム追加
//        auto v_sync = angularVelocitySync(frame);
//        cout << "frame:" << frame << " v_sync:" << v_sync.transpose() << endl;
//        raw_quaternion_with_margin.push_back((raw_quaternion_with_margin.back()*vsp::RotationQuaternion(v_sync*this->T_video)).normalized());
//    }
//    return raw_quaternion_with_margin;
//}

const Eigen::MatrixXd &vsp::filteredData(){
    if(is_filtered){
        return filtered_angle;
    }else{
        int full_tap_length = filter_coeff.rows();
        filtered_angle.resize(raw_angle.rows()-(full_tap_length-1),3);

        for(int i=0,e=filtered_angle.cols();i<e;++i){
            filtered_angle.block(i,0,1,3) = filter_coeff.transpose() * raw_angle.block(i,0,full_tap_length,3);
        }

        is_filtered = true;
        return filtered_angle;
    }
}



Eigen::Quaternion<double> vsp::toRawQuaternion(uint32_t frame){
//    int half_tap_length = filter_coeff.rows()/2;
    Eigen::VectorXd w = raw_angle.block(frame,0,1,3).transpose();
    double theta = w.norm();//回転角度を計算、normと等しい
    //0割を回避するためにマクローリン展開
    //Order: w, x, y, z.
    if(theta > EPS){
        Eigen::VectorXd n = w.normalized();//w * (1.0/theta);//単位ベクトルに変換
        Eigen::VectorXd n_sin_theta_2 = n * sin(theta*0.5);
        return Eigen::Quaternion<double>(cos(theta*0.5),n_sin_theta_2[0],n_sin_theta_2[1],n_sin_theta_2[2]);
    }else{
        return Eigen::Quaternion<double>(1.0,0.5*w[0],0.5*w[1],0.5*w[2]);
    }
}

Eigen::Quaternion<double> vsp::toFilteredQuaternion(uint32_t frame){
//    int half_tap_length = filter_coeff.rows()/2;
    Eigen::VectorXd w = filtered_angle.block(frame,0,1,3).transpose();
    double theta = w.norm();//回転角度を計算
    //0割を回避するためにマクローリン展開
    //Order: w, x, y, z.
    if(theta > EPS){
        Eigen::VectorXd n = w.normalized();//w * (1.0/theta);//単位ベクトルに変換
        Eigen::VectorXd n_sin_theta_2 = n * sin(theta*0.5);
        return Eigen::Quaternion<double>(cos(theta*0.5),n_sin_theta_2[0],n_sin_theta_2[1],n_sin_theta_2[2]);
    }else{
        return Eigen::Quaternion<double>(1.0,0.5*w[0],0.5*w[1],0.5*w[2]);
    }
}

Eigen::Quaternion<double> vsp::toDiffQuaternion2(uint32_t frame){
    Eigen::MatrixXd buf = raw_quaternion.row(frame);
    Eigen::Quaterniond filtered = Eigen::QuaternionMapAlignedd(buf.data());
    Eigen::MatrixXd buf2 = filtered_quaternion.row(frame);
    Eigen::Quaterniond raw = Eigen::QuaternionMapAlignedd(buf2.data());
//    std::cout << filtered.coeffs().transpose() << std::endl;
    return filtered.conjugate()*raw;
}

Eigen::Quaternion<double> vsp::toDiffQuaternion(uint32_t frame){
    return this->toFilteredQuaternion(frame).conjugate()*this->toRawQuaternion(frame);
}

Eigen::VectorXd vsp::getKaiserWindow(uint32_t tap_length, uint32_t alpha, bool swap){
    Eigen::VectorXd window = Eigen::VectorXd::Zero(tap_length);

    if(tap_length % 2){ //奇数
        int32_t L = tap_length/2;
        for(int32_t n=-L,e=L;n<=e;++n){
            window[n+L] = boost::math::cyl_bessel_i(0.0,alpha*sqrt(1.0-pow((double)n/(double)L,2.0)))
                    /boost::math::cyl_bessel_i(0.0,alpha);
        }
    }else{  //偶数
        int32_t L = tap_length/2;
        for(int32_t n=-L,e=L;n<e;++n){//異なる終了条件
            window[n+L] = boost::math::cyl_bessel_i(0.0,alpha*sqrt(1.0-pow((double)n/(double)L,2.0)))
                    /boost::math::cyl_bessel_i(0.0,alpha);
        }
    }

    if(true == swap){
        Eigen::VectorXd buff2(window.rows());
        buff2.block(0,0,window.rows()/2,1) = window.block(window.rows()/2,0,window.rows()/2,1);
        buff2.block(window.rows()/2,0,window.rows()-window.rows()/2,1) = window.block(0,0,window.rows()-window.rows()/2,1);

        return buff2;
    }else{
        return window;
    }
}

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
    Eigen::FFT<double> fft;
    fft.inv(time_vector,frequency_vector);
    Eigen::VectorXd time_eigen_vector = Eigen::Map<Eigen::VectorXd>(&time_vector[0],time_vector.size());
    time_eigen_vector = time_eigen_vector.array() * getKaiserWindow(N,alpha).array();
    Eigen::Map<Eigen::VectorXd>(&time_vector[0],time_vector.size()) = time_eigen_vector;
    fft.fwd(frequency_vector,time_vector);

    return frequency_vector;
}

Eigen::VectorXd vsp::getKaiserWindowWithZeros(int32_t data_length, double alpha, int32_t window_length){
    int32_t length_diff = data_length - window_length;
    Eigen::VectorXd kw = getKaiserWindow(window_length,alpha);
    Eigen::VectorXd kaiser_window_put_zeros = Eigen::VectorXd::Zero(data_length);
    kaiser_window_put_zeros.block(0,0,window_length/2,1)            = kw.block(0,0,window_length/2,1);
    kaiser_window_put_zeros.block(window_length/2,0,length_diff,1)  = Eigen::VectorXd::Zero(length_diff);
    kaiser_window_put_zeros.block(window_length/2 + length_diff,0,window_length-window_length/2,1)
            = kw.block(window_length/2,0,window_length-window_length/2,1);
    //ここに追記
    kaiser_window_put_zeros = kaiser_window_put_zeros.array() / kaiser_window_put_zeros.sum();
    return kaiser_window_put_zeros;
}

Eigen::VectorXcd vsp::getKaiserWindowWithZerosFrequencyCoeff(int32_t data_length, double alpha, int32_t window_length){
    Eigen::FFT<double> fft;
    return fft.fwd(getKaiserWindowWithZeros(data_length,alpha,window_length));
}

void vsp::Angle2CLerpedFrequency(double fs, double fc, const Eigen::MatrixXd &raw_angle, Eigen::MatrixXcd &freq_vectors){
    static Eigen::FFT<double> fft;
    int32_t clerp_length = fs / fc * 20.0;

    //先頭と末尾は繰り返しで不連続になってしまうので、DFT LPFを適用する前に、CLerpでなめらかに繋いでおく
    Eigen::MatrixXd raw_angle_2(raw_angle.rows()+clerp_length,raw_angle.cols());
    raw_angle_2.block(0,0,raw_angle.rows(),raw_angle.cols()) = raw_angle;
    raw_angle_2.block(raw_angle.rows(),0,clerp_length,raw_angle.cols()) = CLerp(raw_angle.block(raw_angle.rows()-1,0,1,raw_angle.cols()),raw_angle.block(0,0,1,raw_angle.cols()),clerp_length);

    freq_vectors.resize(raw_angle_2.rows(),raw_angle_2.cols());

    //Apply LPF to each x, y and z axis.
    for(int i=0,e=raw_angle_2.cols();i<e;++i){
        freq_vectors.col(i) = fft.fwd(raw_angle_2.col(i));
    }
}

void vsp::Frequency2Angle(Eigen::MatrixXcd &frequency_vector_, Eigen::MatrixXd &angle_){
    static Eigen::FFT<double> fft;
    angle_.resize(frequency_vector_.rows(),frequency_vector_.cols());
    for(int i=0,e=frequency_vector_.cols();i<e;++i){
        angle_.col(i).noalias() = fft.inv(frequency_vector_.col(i));
    }
}

Eigen::MatrixXd &vsp::filteredDataDFT(){
    assert(is_filtered == true);
    return filtered_angle;
}

Eigen::MatrixXd &vsp::filteredQuaternion(uint32_t alpha, double fs, double fc){
    if(true == quaternion_is_filtered){
        return filtered_quaternion;
    }else{
        this->fs = fs;
        this->fc = fc;
        {
            int32_t half_filter_tap_length = floor(this->filter_tap_length*0.5);

            auto q = raw_quaternion_with_margin;
            filtered_quaternion.resize(video_frames+2,4);
            Eigen::Quaterniond qo,q_center;
            Eigen::VectorXd kaiser_window = getKaiserWindow(filter_tap_length,alpha,false );
            kaiser_window = Eigen::VectorXd::Zero(filter_tap_length);
            kaiser_window(kaiser_window.rows()/2) = 1.0;
            //総和が1になるように調整
            kaiser_window.array() /= kaiser_window.sum();
            Eigen::MatrixXd exponential_map;
            exponential_map.resize(filter_tap_length,3);
            for(int i=0,e=filtered_quaternion.rows();i<e;++i){

                //1.タップ長分、クォータニオンを集める
                std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond>> buff;
                std::copy(q.begin()+i,q.begin()+i+filter_tap_length,back_inserter(buff));
                //2.回転角を変換
                q_center = buff[buff.size()/2];
                for(auto &el:buff){
                    el=(q_center.conjugate()*el).normalized();
                }
                //3.Exponentialを計算

                for(int k=0,ek=buff.size();k<ek;++k){
                    exponential_map.row(k) = Quaternion2Vector(buff[k]).transpose();
                }
                //3.FIRフィルタ適用
//                Eigen::VectorXd kaiser_window = getKaiserWindow(filter_tap_length,alpha,false );
                //総和が1になるように調整
//                kaiser_window.array() /= kaiser_window.sum();
                exponential_map.array().colwise() *= kaiser_window.array();
                //4.Logを計算
                Eigen::Vector3d temp = exponential_map.colwise().sum().transpose();
                qo = Vector2Quaternion<double>(temp);
                //5.元の座標系に戻す もしかして戻さなくても、差分のベクトルが得られている？？？？
                qo = (q_center*qo).normalized();
                filtered_quaternion.row(i) = qo.coeffs().transpose();
            }


        }
        if(0){
            Eigen::MatrixXcd clerped_quaternion_vectors;
            Angle2CLerpedFrequency(fs,fc,raw_quaternion,clerped_quaternion_vectors);
            Eigen::VectorXcd filter_coeff_vector = getKaiserWindowWithZerosFrequencyCoeff(clerped_quaternion_vectors.rows(),100.0,399);
            //Apply LPF to each w, x, y and z axis.
            for(int i=0,e=clerped_quaternion_vectors.cols();i<e;++i){
                clerped_quaternion_vectors.col(i) = filter_coeff_vector.array() * clerped_quaternion_vectors.col(i).array();
            }

            Frequency2Angle(clerped_quaternion_vectors,filtered_quaternion);

            //ここで末尾の余白を削除
            Eigen::MatrixXd buf = filtered_quaternion.block(0,0,raw_quaternion.rows(),raw_quaternion.cols());
            quaternion_is_filtered = true;

            //θ/2からcos(θ/2)やn・sin(θ/2)にもどす
            buf.block(0,0,buf.rows(),3) = buf.block(0,0,buf.rows(),3).array().colwise() * buf.block(0,0,buf.rows(),3).rowwise().norm().array().sin();
            buf.block(0,3,buf.rows(),1) = buf.block(0,3,buf.rows(),1).array().cos();

            //正規化
            buf.array().colwise() /= buf.rowwise().norm().array();

            //        std::cout << "norm:\r\n" << buf.rowwise().norm();
            //        std::cout << "buf:\r\n" << buf;
            filtered_quaternion = buf;
        }
        quaternion_is_filtered = true;
        return filtered_quaternion;
    }
}

const Eigen::MatrixXd &vsp::filteredDataDFT(double fs, double fc){
    //TODO 最初に条件分岐を追加し、is_filterd == trueなら計算を回避させる
    if(is_filtered && (this->fs == fs) && (this->fc == fc)){
        return filtered_angle;
    }else{
        this->fs = fs;
        this->fc = fc;

        Eigen::MatrixXcd clerped_freq_vectors;
        Angle2CLerpedFrequency(fs,fc,raw_angle,clerped_freq_vectors);
        Eigen::VectorXcd filter_coeff_vector = getKaiserWindowWithZerosFrequencyCoeff(clerped_freq_vectors.rows(),100.0,200);
        //Apply LPF to each x, y and z axis.
        for(int i=0,e=clerped_freq_vectors.cols();i<e;++i){
            clerped_freq_vectors.col(i) = filter_coeff_vector.array() * clerped_freq_vectors.col(i).array();
        }

        Frequency2Angle(clerped_freq_vectors,filtered_angle);

        //ここで末尾の余白を削除
        Eigen::MatrixXd buf = filtered_angle.block(0,0,raw_angle.rows(),raw_angle.cols());
        is_filtered = true;
        filtered_angle = buf;
        return filtered_angle;
    }
}

const Eigen::MatrixXd &vsp::filteredDataDFTTimeDomainOptimize(double fs, double fc, const Eigen::MatrixXd &coeff){
    this->fs = fs;
    this->fc = fc;

    Eigen::MatrixXcd clerped_freq_vectors;
    Eigen::MatrixXd corrected_angle = raw_angle;

    int32_t period = fs/fc;
    assert(coeff.cols() == raw_angle.cols());
//    assert(coeff.rows() == raw_angle.rows()/period);

    Eigen::MatrixXd linear_interporate_coeff = Eigen::MatrixXd::Zero(period,2);

    linear_interporate_coeff.col(0) = Eigen::VectorXd::LinSpaced(linear_interporate_coeff.rows()+1,0.0,1.0).block(0,0,linear_interporate_coeff.rows(),1);
    linear_interporate_coeff.col(1) = Eigen::VectorXd::LinSpaced(linear_interporate_coeff.rows()+1,1.0,0.0).block(0,0,linear_interporate_coeff.rows(),1);

    //線形補間で加算する
    for(int i=0,e=raw_angle.rows()/period;i<e;++i){
        corrected_angle.block(i*period,0,period,3) += linear_interporate_coeff * coeff.block(i,0,2,3);
    }
    //時間波形の長さが、periodの整数倍でない時、別途対応。殆どのケースはこれになる。
    {
        int32_t rest = raw_angle.rows()%period;
        int32_t i = raw_angle.rows()/period;
        if(0 != rest){
            corrected_angle.block(i*period,0,rest,3) += linear_interporate_coeff.block(0,0,rest,2) * coeff.block(i,0,2,3);
        }
    }

    Angle2CLerpedFrequency(fs,fc,corrected_angle,clerped_freq_vectors);
    //Eigen::VectorXcd filter_coeff_vector = getLPFFrequencyCoeff(clerped_freq_vectors.rows(),8,fs,fc).array();
    Eigen::VectorXcd filter_coeff_vector = getKaiserWindowWithZerosFrequencyCoeff(clerped_freq_vectors.rows(),100.0,200);

    //Apply LPF to each x, y and z axis.
    for(int i=0,e=clerped_freq_vectors.cols();i<e;++i){
        clerped_freq_vectors.col(i) = filter_coeff_vector.array() * clerped_freq_vectors.col(i).array();
    }

    Frequency2Angle(clerped_freq_vectors,filtered_angle);

    //ここで末尾の余白を削除
    Eigen::MatrixXd buf = filtered_angle.block(0,0,raw_angle.rows(),raw_angle.cols());
    is_filtered = true;
    filtered_angle = buf;
    return filtered_angle;
}

Eigen::MatrixXd vsp::CLerp(Eigen::MatrixXd start, Eigen::MatrixXd end, int32_t num){
    assert(start.cols() == end.cols());
    assert(start.rows() == end.rows());
    Eigen::VectorXd phase = Eigen::ArrayXd::LinSpaced(num,0,M_PI);
    Eigen::VectorXd cos_phase = phase.array().cos();
    return cos_phase * (-(end - start)*0.5) + Eigen::VectorXd::Ones(phase.rows()) * (start + end) * 0.5;
}

Eigen::VectorXd vsp::getRollingVectorError(){
    double eps = 1.0e-5;

    std::vector<float> vecPorigonn_uv;

    auto func = [&](int32_t frame, double ratio){
        Eigen::Quaternion<double> prevQ;
        Eigen::Quaternion<double> currQ;
        Eigen::Quaternion<double> nextQ;
        if(0 == frame){
            currQ = Vector2Quaternion<double>(ratio*Quaternion2Vector(Vector2Quaternion<double>(filtered_angle.row(frame  ).transpose()).conjugate() * Vector2Quaternion<double>(raw_angle.row(frame  ).transpose())));
            prevQ = currQ;
            nextQ = Vector2Quaternion<double>(ratio*Quaternion2Vector(Vector2Quaternion<double>(filtered_angle.row(frame+1).transpose()).conjugate() * Vector2Quaternion<double>(raw_angle.row(frame+1).transpose())));
        }else if((raw_angle.rows()-1) == frame){
            prevQ = Vector2Quaternion<double>(ratio*Quaternion2Vector(Vector2Quaternion<double>(filtered_angle.row(frame-1).transpose()).conjugate() * Vector2Quaternion<double>(raw_angle.row(frame-1).transpose())));
            currQ = Vector2Quaternion<double>(ratio*Quaternion2Vector(Vector2Quaternion<double>(filtered_angle.row(frame  ).transpose()).conjugate() * Vector2Quaternion<double>(raw_angle.row(frame  ).transpose())));
            nextQ = currQ;
        }else{
            prevQ = Vector2Quaternion<double>(ratio*Quaternion2Vector(Vector2Quaternion<double>(filtered_angle.row(frame-1).transpose()).conjugate() * Vector2Quaternion<double>(raw_angle.row(frame-1).transpose())));
            currQ = Vector2Quaternion<double>(ratio*Quaternion2Vector(Vector2Quaternion<double>(filtered_angle.row(frame  ).transpose()).conjugate() * Vector2Quaternion<double>(raw_angle.row(frame  ).transpose())));
            nextQ = Vector2Quaternion<double>(ratio*Quaternion2Vector(Vector2Quaternion<double>(filtered_angle.row(frame+1).transpose()).conjugate() * Vector2Quaternion<double>(raw_angle.row(frame+1).transpose())));
        }
        getDistortUnrollingContour(
                    prevQ,
                    currQ,
                    nextQ,
//                    division_x,
//                    division_y,
//                    TRollingShutter,
//                    IK,
//                    matIntrinsic,
//                    image_width,
//                    image_height,
                    vecPorigonn_uv
//                    zoom
                    );
        return check_warp(vecPorigonn_uv);
    };

    Eigen::VectorXd retval = Eigen::VectorXd::Zero(raw_angle.rows());

    double error = 0.0;
    for(int32_t frame=0,e=retval.rows();frame<e;++frame){
        int count=0;
        double a=0.0;
        double b=1.0;
        if(func(frame,1.0)==true){
            retval[frame] = 0.0;
            continue;
        }
        double m;
        do{
            count++;
            m=(a+b)/2.0;
            if(func(frame,m)^func(frame,a)){
                b=m;
            }else{
                a=m;
            }
            if(count == 1000){
//                cout << "frame:" << frame << " 収束失敗" << endl;
                break;
            }
        }while(!(abs(a-b)<eps));
//        cout <<  "frame:" << frame << " " << count << "回で収束" << endl;

        error = (Quaternion2Vector(Vector2Quaternion<double>(filtered_angle.row(frame  ).transpose()).conjugate() * Vector2Quaternion<double>(raw_angle.row(frame  ).transpose())).norm())*(1-m);
        retval[frame] = error;
    }
    return retval;
}

void vsp::MatrixXcd2VectorXd(const Eigen::MatrixXcd &src, Eigen::VectorXd &dst){
    assert(!src.IsRowMajor);//メモリ配置はcol majorでなければならない
    assert(0 == dst.rows()%(2*src.cols()));//2とsrc列数の公倍数。2はMatrixXcdの1要素がimagとrealの2つの要素からなることに起因している
    int32_t row_elements_to_copy = dst.rows()/src.cols();//imaginaly and real number
    int32_t col_copy = src.cols();
    for(int i=0;i<col_copy;++i){
        memcpy((double*)dst.data()+i*row_elements_to_copy,(double*)src.data()+i*src.rows()*2,row_elements_to_copy*sizeof(double));
    }
}

void vsp::VectorXd2MatrixXcd(const Eigen::VectorXd &src, Eigen::MatrixXcd &dst){
    assert(!dst.IsRowMajor);//メモリ配置はcol majorでなければならない
    assert(0 == src.rows()%(2*dst.cols()));//2とdst列数の公倍数。2はMatrixXcdの1要素がimagとrealの2つの要素からなることに起因している
    int32_t row_elements_to_copy = src.rows()/dst.cols();//imaginaly and real number
    int32_t col_copy = dst.cols();
    for(int i=0;i<col_copy;++i){
        memcpy((double*)dst.data()+i*dst.rows()*2,(double*)src.data()+i*row_elements_to_copy,row_elements_to_copy*sizeof(double));
    }

}

Eigen::Vector3d vsp::angularVelocitySync(/*std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> &angularVelocityIn60Hz,
                                    double T_video,
                                    double T_av,
                                    double frame_offset,*/
                                    int32_t frame){
    double dframe = (frame + frame_offset) * T_video / T_angular_velocity;
    int i = floor(dframe);
    double decimalPart = dframe - (double)i;
    //領域外にはみ出した時は、末端の値で埋める
    if(i<0){
        return angular_velocity[0];
    }else if(angular_velocity.size()<=(i+1)){
        return angular_velocity.back();
    }else{
        return angular_velocity[i]*(1.0-decimalPart)+angular_velocity[i+1]*decimalPart;
    }
}
