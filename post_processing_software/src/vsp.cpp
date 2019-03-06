#include "vsp.h"
#include <boost/math/special_functions/bessel.hpp>

GLFWwindow* window;

vsp::vsp(/*vector<Eigen::Quaternion<T>> &angle_quaternion,*/
         int32_t division_x,
         int32_t division_y,
         CameraInformation camera_info,
         double zoom,
         std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> &angular_velocity,
         double T_video,
         double T_angular_velocity,
         double frame_offset,
         int32_t video_frames,
         int32_t filter_tap_length):camera_info_(camera_info){
    is_filtered=false;
    this->division_x = division_x;
    this->division_y = division_y;
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
        Eigen::Vector3d v_sync = angularVelocitySync(frame);
        raw_quaternion_with_margin.push_back((raw_quaternion_with_margin.back()*vsp::RotationQuaternion(v_sync*this->T_video)).normalized());
    }

    int32_t half_tap_length = filter_tap_length/2;
    raw_quaternion.resize(video_frames+2,4);
    for(int i=0,e=raw_quaternion.rows();i<e;++i){
        raw_quaternion.row(i)=raw_quaternion_with_margin[i+half_tap_length].coeffs().transpose();
    }

    is_filtered = false;
}

vsp::vsp(int32_t division_x,
        int32_t division_y,
        CameraInformation camera_info,
    double zoom,
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> &angular_velocity,
    double T_video,
    double T_angular_velocity,
    double frame_offset,
    int32_t video_frames,
    int32_t filter_tap_length,
    Eigen::MatrixXd &raw_quaternion,
    Eigen::MatrixXd &filtered_quaternion)
    :is_filtered(false),division_x(division_x),division_y(division_y),
      camera_info_(camera_info),zoom(zoom),
      angular_velocity(angular_velocity),T_video(T_video),T_angular_velocity(T_angular_velocity),
      frame_offset(frame_offset),video_frames(video_frames),filter_tap_length(filter_tap_length),
      raw_quaternion(raw_quaternion),filtered_quaternion_(filtered_quaternion)
{

}

void vsp::setParam(double fs, double fc){
    this->fs = fs;
    this->fc = fc;
}

Eigen::Quaternion<double> vsp::RotationQuaternion(double theta, Eigen::Vector3d n){
    //nを規格化してから計算する
    Eigen::Vector3d n_sin_theta_2 = n.normalized()*sin(theta/2.0);
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
    return RotationQuaternion(theta, w.normalized());
}

const Eigen::MatrixXd &vsp::toQuaternion(){
    return raw_quaternion;
}
Eigen::Quaternion<double> vsp::toDiffQuaternion2(uint32_t frame){
    Eigen::MatrixXd buf = raw_quaternion.row(frame);
    Eigen::Quaterniond raw = Eigen::QuaternionMapAlignedd(buf.data());
    Eigen::MatrixXd buf2 = filtered_quaternion_.row(frame);
    Eigen::Quaterniond filtered = Eigen::QuaternionMapAlignedd(buf2.data());
    //    std::cout << filtered.coeffs().transpose() << std::endl;
    return filtered.conjugate()*raw;
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

//TODO:なんかframe位置ずれてそう。大丈夫か？
Eigen::Quaterniond vsp::filteredQuaternion(int32_t alpha,int32_t frame){

    std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond>> &q = raw_quaternion_with_margin;
    filtered_quaternion_.resize(video_frames+2,4);
    Eigen::Quaterniond qo,q_center;
    Eigen::VectorXd kaiser_window = getKaiserWindow(filter_tap_length,alpha,false );
    //総和が1になるように調整
    kaiser_window.array() /= kaiser_window.sum();
    Eigen::MatrixXd exponential_map;
    exponential_map.resize(filter_tap_length,3);
//    for(int frame=0,e=filtered_quaternion.rows();frame<e;++frame){

        //1.タップ長分、クォータニオンを集める
        std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond>> buff;
        std::copy(q.begin()+frame,q.begin()+frame+filter_tap_length,back_inserter(buff));
        //2.回転角を変換
        q_center = buff[buff.size()/2];
        //                q_center = Eigen::Quaterniond(1.0,0,0,0);
        for(auto &el:buff){
            el=(q_center.conjugate()*el).normalized();
        }
        //3.Exponentialを計算

        for(int k=0,ek=buff.size();k<ek;++k){
            exponential_map.row(k) = Quaternion2Vector(buff[k]).transpose();
        }
        //3.FIRフィルタ適用
        exponential_map.array().colwise() *= kaiser_window.array();
        //4.Logを計算
        Eigen::Vector3d filtered_vector = exponential_map.colwise().sum().transpose();
        qo = Vector2Quaternion<double>(filtered_vector);
        //5.元の座標系に戻す もしかして戻さなくても、差分のベクトルが得られている？？？？
        qo = (q_center*qo).normalized();
//        return qo.coeffs().transpose();
        return qo;
//    }
}

Eigen::MatrixXd &vsp::filteredQuaternion(uint32_t alpha){
    if(true == quaternion_is_filtered){
        return filtered_quaternion_;
    }else{
        std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond>> &q = raw_quaternion_with_margin;
        filtered_quaternion_.resize(video_frames+2,4);
        Eigen::Quaterniond qo,q_center;
        Eigen::VectorXd kaiser_window = getKaiserWindow(filter_tap_length,alpha,false );
        //            kaiser_window = Eigen::VectorXd::Zero(filter_tap_length);
        //            kaiser_window(kaiser_window.rows()/2) = 1.0;
        //総和が1になるように調整
        kaiser_window.array() /= kaiser_window.sum();
        Eigen::MatrixXd exponential_map;
        exponential_map.resize(filter_tap_length,3);
        for(int i=0,e=filtered_quaternion_.rows();i<e;++i){

            //1.タップ長分、クォータニオンを集める
            std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond>> buff;
            std::copy(q.begin()+i,q.begin()+i+filter_tap_length,back_inserter(buff));
            //2.回転角を変換
            q_center = buff[buff.size()/2];
            //                q_center = Eigen::Quaterniond(1.0,0,0,0);
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
            filtered_quaternion_.row(i) = qo.coeffs().transpose();
        }



        quaternion_is_filtered = true;
        return filtered_quaternion_;
    }
}

Eigen::MatrixXd &vsp::filteredQuaternion(Eigen::VectorXd &filter_coefficients){
    std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond>> &q = raw_quaternion_with_margin;
    filtered_quaternion_.resize(video_frames+2,4);
    Eigen::Quaterniond qo,q_center;
    Eigen::MatrixXd exponential_map;
    exponential_map.resize(filter_tap_length,3);
    for(int i=0,e=filtered_quaternion_.rows();i<e;++i){

        //1.タップ長分、クォータニオンを集める
        std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond>> buff;
        std::copy(q.begin()+i,q.begin()+i+filter_tap_length,back_inserter(buff));
        //2.回転角を変換
        q_center = buff[buff.size()/2];
        //                q_center = Eigen::Quaterniond(1.0,0,0,0);
        for(auto &el:buff){
            el=(q_center.conjugate()*el).normalized();
        }
        //3.Exponentialを計算

        for(int k=0,ek=buff.size();k<ek;++k){
            exponential_map.row(k) = Quaternion2Vector(buff[k]).transpose();
        }
        //3.FIRフィルタ適用
        Eigen::VectorXd kaiser_window = getKaiserWindow(filter_tap_length,filter_coefficients[i],false);
        //総和が1になるように調整
        kaiser_window.array() /= kaiser_window.sum();
        exponential_map.array().colwise() *= kaiser_window.array();
        //4.Logを計算
        Eigen::Vector3d temp = exponential_map.colwise().sum().transpose();
        qo = Vector2Quaternion<double>(temp);
        //5.元の座標系に戻す もしかして戻さなくても、差分のベクトルが得られている？？？？
        qo = (q_center*qo).normalized();
        filtered_quaternion_.row(i) = qo.coeffs().transpose();
    }



    quaternion_is_filtered = true;
    return filtered_quaternion_;
}


Eigen::MatrixXd vsp::CLerp(Eigen::MatrixXd start, Eigen::MatrixXd end, int32_t num){
    assert(start.cols() == end.cols());
    assert(start.rows() == end.rows());
    Eigen::VectorXd phase = Eigen::ArrayXd::LinSpaced(num,0,M_PI);
    Eigen::VectorXd cos_phase = phase.array().cos();
    return cos_phase * (-(end - start)*0.5) + Eigen::VectorXd::Ones(phase.rows()) * (start + end) * 0.5;
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

Eigen::Vector3d vsp::angularVelocitySync(int32_t frame){
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

Eigen::MatrixXd &vsp::getRawQuaternion(){
    return raw_quaternion;
}

Eigen::MatrixXd &vsp::getFilteredQuaternion(){
    return filtered_quaternion_;
}

#define TEST2D

Eigen::Quaternion<double> vsp::toDiffQuaternion2(int32_t filter_strength, uint32_t frame){
    Eigen::MatrixXd buf = raw_quaternion.row(frame);
    Eigen::Quaterniond raw = Eigen::QuaternionMapAlignedd(buf.data());
    Eigen::Quaterniond filtered = filteredQuaternion(filter_strength,frame);
//     = Eigen::QuaternionMapAlignedd(buf2.data());
    return filtered.conjugate()*raw;
}

bool vsp::hasBlackSpace(int32_t filter_strength, int32_t frame){
    std::vector<float> vecPorigonn_uv;
    Eigen::Quaternion<double> prevQ;
    Eigen::Quaternion<double> currQ;
    Eigen::Quaternion<double> nextQ;
    if(0 == frame){
        currQ = toDiffQuaternion2(filter_strength,frame);
        prevQ = currQ;
        nextQ = toDiffQuaternion2(filter_strength,frame+1);
    }else if((raw_quaternion.rows()-1) == frame){
        prevQ = toDiffQuaternion2(filter_strength,frame-1);
        currQ = toDiffQuaternion2(filter_strength,frame);
        nextQ = currQ;
    }else{
        prevQ = toDiffQuaternion2(filter_strength,frame-1);
        currQ = toDiffQuaternion2(filter_strength,frame);
        nextQ = toDiffQuaternion2(filter_strength,frame+1);
    }
    getDistortUnrollingContour(
                prevQ,
                currQ,
                nextQ,
                vecPorigonn_uv
                );
    return !isPerfectWarp(vecPorigonn_uv);
}

uint32_t vsp::bisectionMethod(int32_t frame, int32_t minimum_filter_strength, int32_t maximum_filter_strength, int max_iteration, uint32_t eps){
    int32_t a = minimum_filter_strength;
    int32_t b = maximum_filter_strength;
    int count = 0;
    int32_t m;
//    while(hasBlackSpace(maximum_filter_strength,frame)){
//        minimum_filter_strength = maximum_filter_strength;
//        maximum_filter_strength *= 2;
//    }
    while((abs(a-b)>eps) && (count++ < max_iteration)){
        m=(a+b)*0.5;
        if(hasBlackSpace(a,frame)^hasBlackSpace(m,frame)){
            b = m;
        }else{
            a = m;
        }
        if(count == max_iteration){
            std::cout << "max_iteration" << std::endl;
        }
    }
    return m;
}

Eigen::VectorXd vsp::calculateFilterCoefficientsWithoutBlackSpaces(int32_t minimum_filter_strength, int32_t maximum_filter_strength){
    Eigen::VectorXd filter_strength(raw_quaternion.rows());
    //Calcurate in all frame
    for(int frame=0,e=filter_strength.rows();frame<e;++frame){
        if(hasBlackSpace(maximum_filter_strength,frame)){
            filter_strength[frame] = maximum_filter_strength;
        }else if(!hasBlackSpace(minimum_filter_strength,frame)){
            filter_strength[frame] = minimum_filter_strength;
        }else{
            filter_strength[frame] = bisectionMethod(frame,minimum_filter_strength,maximum_filter_strength);
        }
    }
//    std::cout << filter_strength << std::endl;
    gradientLimit(filter_strength);

    return(filter_strength);
}

void vsp::gradientLimit(Eigen::VectorXd &input){
    double limited_value = input.head(1)[0];
    for(int i=0,e=input.rows();i<e;++i){
        if(input(i) > limited_value - maximum_gradient_){
            limited_value = input(i);
        }else{
            limited_value -= maximum_gradient_;
            input(i) = limited_value;
        }
    }
    limited_value = input.tail(1)[0];
    for(int i=input.rows()-1;i>=0;--i){
        if(input(i) > limited_value - maximum_gradient_){
            limited_value = input(i);
        }else{
            limited_value -= maximum_gradient_;
            input(i) = limited_value;
        }
    }
}

void vsp::setMaximumGradient(double value){
    maximum_gradient_ = value;
}

int vsp::init_opengl(cv::Size textureSize){
    this->buff = cv::Mat(textureSize.height,textureSize.width,CV_8UC3);
    this->textureSize = textureSize;
    // Initialise GLFW
    if( !glfwInit() )
    {
        fprintf( stderr, "Failed to initialize GLFW\n" );
        getchar();
        return -1;
    }



    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef TEST2D
    glfwWindowHint( GLFW_VISIBLE, 0 );//オフスクリーンレンダリング。
#endif

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

//    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    // Create and compile our GLSL program from the shaders
    /*GLuint */programID = LoadShaders( "TransformVertexShader.vertexshader", "TextureFragmentShader.fragmentshader" );

    // Get a handle for our "MVP" uniform
    /*GLuint */MatrixID = glGetUniformLocation(programID, "MVP");


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
    /*GLuint */FramebufferName = 0;
    glGenFramebuffers(1, &FramebufferName);
    glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);

    // The texture we're going to render to
    GLuint renderedTexture;

    GLuint textures[2];
    glGenTextures(2, textures);
    renderedTexture = textures[0];
    /*GLuint */textureID_0 = textures[1];

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
    glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, camera_info_.width_, camera_info_.height_, 0,GL_RGB, GL_UNSIGNED_BYTE, 0);

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

    // Get a handle for our "myTextureSampler" uniform
    /*GLuint */TextureID  = glGetUniformLocation(programID, "myTextureSampler");

    std::vector<GLfloat> vecTexture;
    for(int j=0;j<division_y;++j){							//jは終了の判定が"<"であることに注意
        double v	= (double)j/division_y*camera_info_.height_;
        double v1	= (double)(j+1)/division_y*camera_info_.height_;
        for(int i=0;i<division_x;++i){
            double u	= (double)i/division_x*camera_info_.width_;
            double u1	= (double)(i+1)/division_x*camera_info_.width_;
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

//    std::vector<GLfloat> vecVtx(vecTexture.size());					//頂点座標

//    GLuint vertexbuffer;
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    //    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);
    glBufferData(GL_ARRAY_BUFFER, vecVtx.size()*sizeof(GLfloat), vecVtx.data(), GL_DYNAMIC_DRAW);

//    GLuint uvbuffer;
    glGenBuffers(1, &uvbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
    //    glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data), g_uv_buffer_data, GL_STATIC_DRAW);
    glBufferData(GL_ARRAY_BUFFER, vecTexture.size()*sizeof(GLfloat), vecTexture.data(), GL_STATIC_DRAW);

    //歪補正の準備
    /*GLuint */nFxyID       = glGetUniformLocation(programID, "normalizedFocalLength");
    /*GLuint */nCxyID       = glGetUniformLocation(programID, "normalizedOpticalCenter");
    /*GLuint */distCoeffID  = glGetUniformLocation(programID, "distortionCoeffs");



    //動画の位置を修正
//    cv::Mat img;

vecVtx.resize(vecTexture.size());

//this->outputStabilizedVideo = outputStabilizedVideo;


}

int vsp::stop_opengl(){
    // Cleanup VBO and shader
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteBuffers(1, &uvbuffer);
    glDeleteProgram(programID);
    glDeleteTextures(1, &TextureID);
    glDeleteVertexArrays(1, &VertexArrayID);
    //ここでTextureID_0をDeleteしなくて大丈夫？
    // Close OpenGL window and terminate GLFW
    glfwTerminate();
}

int vsp::spin_once(int frame,cv::VideoCapture &capture, cv::Mat &simg){
#if MULTITHREAD_CAPTURE
    {
        std::lock_guard<std::mutex> lock(mtvc.mtx);
        img = mtvc.images.front().clone();
        mtvc.images.pop_front();//先頭を削除
    }
#else
    capture >> img;
    if(img.empty()){
        std::cout << "Empty" << std::endl;
        return -1;
    }
#endif

    getDistortUnrollingMapQuaternion(frame,vecVtx);

#ifdef TEST2D
    // Render to our framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
    glViewport(0,0,camera_info_.width_,camera_info_.height_); // Render on the whole framebuffer, complete from the lower left corner to the upper right
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
    float nfxy[] = {(float)(camera_info_.fx_/camera_info_.width_), (float)(camera_info_.fy_/camera_info_.height_)};
    glUniform2fv(nFxyID, 1, nfxy);
    float ncxy[] = {(float)(camera_info_.cx_/camera_info_.width_), (float)(camera_info_.cy_/camera_info_.height_)};
    glUniform2fv(nCxyID, 1, ncxy);
    //        float distcoeffFloat[] = {(float)(matDist.at<double>(0,0)),(float)(matDist.at<double>(0,1)),(float)(matDist.at<double>(0,2)),(float)(matDist.at<double>(0,3))};
    float distcoeffFloat[] = {(float)(camera_info_.inverse_k1_  ),(float)(camera_info_.inverse_k2_),(float)(camera_info_.inverse_p1_),(float)(camera_info_.inverse_p2_)};
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

#if 1
    ////////////////////
//    cv::Mat simg(cv::Size(camera_info_.width_,camera_info_.height_),CV_8UC3);
    simg = cv::Mat(cv::Size(camera_info_.width_,camera_info_.height_),CV_8UC3);
    //~ glReadBuffer(GL_FRONT);//読み取るOpenGLのバッファを指定 GL_FRONT:フロントバッファ GL_BACK:バックバッファ


    //        glReadBuffer(GL_BACK);//読み取るOpenGLのバッファを指定 GL_FRONT:フロントバッファ GL_BACK:バックバッファ
    // OpenGLで画面に描画されている内容をバッファに格納
    glReadPixels(
                0,					//読み取る領域の左下隅のx座標
                0,					//読み取る領域の左下隅のy座標 //0 or getCurrentWidth() - 1
                camera_info_.width_,				//読み取る領域の幅
                camera_info_.height_,				//読み取る領域の高さ
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

    cv::Rect roi_dst(0,camera_info_.height_/4,camera_info_.width_/2,camera_info_.height_/2);
    cv::Rect roi_src(camera_info_.width_/2,camera_info_.height_/4,camera_info_.width_/2,camera_info_.height_/2);

    char pressed_key = cv::waitKey(1);
    if(pressed_key != -1){
        key = pressed_key;
    }

    switch (key) {
    case KEY_ORIGINAL:
        cv::putText(img, "Original", cv::Point(625,150+camera_info_.height_*0.75),cv::FONT_HERSHEY_SIMPLEX,5, cv::Scalar(0,255,255),12,cv::LINE_AA);
        cv::imshow("Preview",img);
        break;

    case KEY_STABILIZED:
        cv::putText(simg, "Stabilized", cv::Point(625,150+camera_info_.height_*0.75),cv::FONT_HERSHEY_SIMPLEX,5, cv::Scalar(0,255,255),12,cv::LINE_AA);
        cv::imshow("Preview",simg);
        break;
    case KEY_QUIT:
        return -1;
        break;
    default: //KEY_STABILIZED

        static cv::Mat sidebyside(cv::Size(camera_info_.width_,camera_info_.height_),CV_8UC3,cv::Scalar(0));
        static bool first_time = true;
        if(first_time){
            cv::putText(sidebyside, "Original", cv::Point(200,150+camera_info_.height_*0.75),cv::FONT_HERSHEY_SIMPLEX,5, cv::Scalar(0,255,255),12,cv::LINE_AA);
            cv::putText(sidebyside, "Stabilized", cv::Point(1050,150+camera_info_.height_*0.75),cv::FONT_HERSHEY_SIMPLEX,5, cv::Scalar(0,255,255),12,cv::LINE_AA);
            first_time = false;
        }


        cv::resize(simg,sidebyside(roi_src),cv::Size(),0.5,0.5,cv::INTER_LINEAR);
        cv::resize(img,sidebyside(roi_dst),cv::Size(),0.5,0.5,cv::INTER_LINEAR);



        cv::imshow("Preview",sidebyside);

        break;
    }



//    if(outputStabilizedVideo){
//        std::lock_guard<std::mutex> lock(buffer.mtx);
//        buffer.images.push_back(cv::Mat());
//        buffer.images.back() = simg.clone();
//    }


    /////////////////////

    // Swap buffers
    //        glfwSwapBuffers(window);
    glfwPollEvents();

    if(glfwGetKey(window, GLFW_KEY_ESCAPE ) == GLFW_PRESS ||
            glfwWindowShouldClose(window) != 0 ){
        return 1;
    }
    return 0;
}

bool vsp::ok(){
    return key != KEY_QUIT;
}
