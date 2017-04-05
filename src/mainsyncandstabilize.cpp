#include <stdio.h>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

//Quartanion
#include <boost/math/quaternion.hpp>
using namespace boost::math;

// Include standard headers
#include <stdio.h>
#include <stdlib.h>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <glfw3.h>
GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include <common/shader.hpp>
#include <common/texture.hpp>
#include <common/controls.hpp>

#include "mINIRead.hpp"

/**
* @brief カメラ行列設定を記録したテキストファイルを読み込みます
* @param [in] FileName		テキストファイルの名前。例:"intrinsic.txt"
* @param [out] Values		読み取った値を書き込む行列。
* @retval 0で正常
**/
//int ReadIntrinsicsParams(const char* FileName, cv::Mat &Values){
//    std::ifstream ifs(FileName);//CSVファイルを開く
//    if(!ifs){
//        printf("エラー：カメラ行列を記録した%sが見つかりません\nプログラムを終了します\n",FileName);//TODO:abotrせずに開けなかったことを上に返すようにして、その場合は推定値を使うようにする
//        abort();
//    }
//    std::string str;
//    Values = cv::Mat::zeros(3,3,CV_64F);//matを初期化

//    int j=0;
//    while(getline(ifs,str)){//1行ずつ読み込む
//        std::string token;
//        std::istringstream stream(str);

//        int i=0;
//        while(getline(stream, token, ' ')){
//            Values.at<double>(j,i++) = (double)stof(token);//値を保存
//            if(i>3){
//                printf("エラー:カメラ行列のパラメータファイルintrinsic.txtの書式が不正なため、プログラムを終了します。\n");
//                abort();
//            }
//        }
//        ++j;
//    }

//    return 0;

//}


/**
* @brief 歪補正パラメータを記録したテキストファイルを読み込みます
* @param [in] FileName		テキストファイルの名前。例:"distortion.txt"
* @param [out] Values		読み取った値を書き込む構造体。
* @retval 0で正常
**/
//template<typename T_dcoef>
//int ReadDistortionParams(const char FileName[], cv::Mat &Values){
//    std::ifstream ifs(FileName);//CSVファイルを開く
//    if(!ifs){
//        printf("エラー：歪パラメータを記録した%sが見つかりません\nプログラムを終了します\n",FileName);//TODO:abotrせずに開けなかったことを上に返すようにして、その場合は推定値を使うようにする
//        abort();
//    }
//    std::string str;
//    Values = cv::Mat::zeros(1,4,CV_64F);//matを初期化

//    int j=0;
//    while(getline(ifs,str)){//1行ずつ読み込む
//        std::string token;
//        std::istringstream stream(str);

//        int i=0;
//        while(getline(stream, token, ' ')){
//            Values.at<double>(0,i++) = (double)stof(token);//値を保存
//        }
//    }
//    return 0;



//    //エラーチェック
//    assert(Values.cols == 5);
//    assert(Values.rows == 1);

//    FILE *fp;
//    const int Num = 5;
//    // テキストファイルを読み込み用に開く
//    if (fopen(FileName, "r") != 0){
//        // ファイルオープンに失敗
//        printf("File Open Error. \"%s\" is not found.\n", FileName);
//        return 1;
//    }

//    double valuesd[5] = { 0.0 };
//    if (fscanf(fp, "%lf %lf %lf %lf %lf", &valuesd[0], &valuesd[1], &valuesd[2], &valuesd[3], &valuesd[4]) == -1){//1行読み込み
//        //エラー処理
//        printf("エラー:歪パラメータファイルの読み込みに失敗しました。\r\n");
//        printf("Check format of \"%s\".\n", FileName);
//        fclose(fp);
//        return 1;
//    }
//    int i;
//    printf("%s\r\n", FileName);
//    for (i = 0; i < Num; i++){
//        //printf("%f%t\n", valuesd[i]);
//        Values.at<double>(0, i) = valuesd[i];//cv::Matに代入
//    }
//    std::cout << Values << std::endl;
//    printf("\r\n");
//    fclose(fp);
//    return 0;

//}

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
    omega = acos(cosom);
    sinom = sin(omega);
    scale0 = sin((1.0 - t) * omega) / sinom;
    scale1 = sin(t * omega) / sinom;

    return scale0 * Qfrom + scale1 * Qto;

}


 /** @brief 補正前の画像座標から、補正後のポリゴンの頂点の位置を表す座標の組を作成
 * @param [in]	Qa	ジャイロの角速度から計算したカメラの方向を表す回転クウォータニオン時系列データ、参照渡し
 * @param [in]	Qf	LPFを掛けて平滑化した回転クウォータニオンの時系列データ、参照渡し
 * @param [in]	m	画面の縦の分割数[ ]
 * @param [in]	n	画面の横の分割数[ ]
 * @param [in]	ti	角度時系列データのサンプル時間[sec]
 * @param [in]	ts	ローリングシャッターの全行取得時間[sec]
 * @param [in]	T	ジャイロ角度のサンプリング周期
 * @param [in]	IK	"逆"歪係数(k1,k2,p1,p2)
 * @param [in]	matIntrinsic	カメラ行列(fx,fy,cx,cy) [pixel]
 * @param [in]	imagesize	フレーム画像のサイズ[pixel]
 * @param [in]	textureSize	テクスチャの大きさ[pixel]
 * @param [out]	vecPorigonn_uv	OpenGLのポリゴン座標(u',v')座標(0~1)の組、歪補正後の画面を分割した時の一つ一つのポリゴンの頂点の組
 * @param [in]	zoom	倍率[]。拡大縮小しないなら1を指定すること。省略可
 **/
template <typename _Tp, typename _Tx> void getDistortUnrollingMap(
    std::vector<quaternion<_Tp>> &Qa,
    std::vector<quaternion<_Tp>> &Qf,
    unsigned int m,
    unsigned int n,
    double ti,
    double ts,
    double T,
    cv::Mat &IK,
    cv::Mat &matIntrinsic,
    cv::Size imgSize,
    cv::Size textureSize,
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

    cv::Mat map(n+1,m+1,CV_64FC2);


    for(int j=0;j<=m;++j){
        //W(t1,t2)を計算
        cv::Mat R;
        //1
        double v = (double)j/m*imgSize.height;

        double tiy = ti + ts*v/imgSize.height;	//ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]
        unsigned int fi = (int)floor(tiy/T);	//ローリングシャッター補正を含むフレーム数の整数部[ ]
        double ff = tiy/T - (double)fi;			//ローリングシャッター補正を含むフレーム数の浮動小数点数部[ ]
        auto SQa = Slerp(Qa[fi],Qa[fi+1],ff);	//オリジナルの角度クウォータニオンに関して球面線形補間

        unsigned int gi = (int)floor(ti/T);		//フレーム数の整数部[ ]
        double gf = ti/T - (double)gi;			//フレーム数の浮動小数点数部[ ]
        auto SQf = Slerp(Qf[gi],Qf[gi+1],gf);	//フィルタ済みの角度クウォータニオンに関して球面線形補間

        Quaternion2Matrix(conj(SQf)*SQa,R);		//ローリングシャッター補正を含む回転行列を計算

        for(int i=0;i<=n;++i){
            double u = (double)i/n*imgSize.width;
            //後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
            cv::Mat p = (cv::Mat_<double>(3,1) << (u- cx)/fx, (v - cy)/fy, 1.0);	//1のポリゴン座標に、K^-1を掛けた結果の３x１行列
            //2
            cv::Mat XYW = R * p;//inv()なし

            double x1 = XYW.at<double>(0, 0)/XYW.at<double>(2, 0);
            double y1 = XYW.at<double>(1, 0)/XYW.at<double>(2, 0);

            double r = sqrt(x1*x1+y1*y1);

            double x2 = x1*(1.0+k1*r*r+k2*r*r*r*r)+2.0*p1*x1*y1+p2*(r*r+2.0*x1*x1);
            double y2 = y1*(1.0+k1*r*r+k2*r*r*r*r)+p1*(r*r+2.0*y1*y1)+2.0*p2*x1*y1;
            double mapx = x2*fx*zoom+cx;
            double mapy = y2*fy*zoom+cy;
            //~ return ;
            //結果をmapに保存
            map.at<cv::Vec2d>(j,i)[0] = mapx;
            map.at<cv::Vec2d>(j,i)[1] = mapy;
            //~ printf("i:%d,j:%d,mapx:%4.3f,mapy:%4.3f\n",i,j,mapx,mapy);
        }
    }

    //3.ポリゴン座標をOpenGLの関数に渡すために順番を書き換える
    vecPorigonn_uv.clear();
    for(int j=0;j<m;++j){//jは終了の判定が"<"であることに注意
        for(int i=0;i<n;++i){
            //GL_QUADでGL側へ送信するポリゴンの頂点座標を準備
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j,i)[0]);//x座標
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j,i)[1]);//y座標
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j+1,i)[0]);//x座標
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j+1,i)[1]);//y座標
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j+1,i+1)[0]);//x座標
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j+1,i+1)[1]);//y座標
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j,i+1)[0]);//x座標
            vecPorigonn_uv.push_back(map.at<cv::Vec2d>(j,i+1)[1]);//y座標
        }
    }




}


int main(int argc, char** argv){
    cv::VideoCapture Capture(argv[1]);//動画をオープン
    assert(Capture.isOpened());
    cv::Size imageSize = cv::Size(Capture.get(CV_CAP_PROP_FRAME_WIDTH),Capture.get(CV_CAP_PROP_FRAME_HEIGHT));//解像度を読む
    double samplingPeriod = 1.0/Capture.get(CV_CAP_PROP_FPS);
    std::cout << "resolution" << imageSize << std::endl;
    std::cout << "samplingPeriod" << samplingPeriod << std::endl;

    //内部パラメータを読み込み
    cv::Mat matIntrinsic;
    ReadIntrinsicsParams("intrinsic.txt",matIntrinsic);
    std::cout << "Camera matrix:\n" << matIntrinsic << "\n" <<  std::endl;

    //歪パラメータの読み込み
    cv::Mat matDist;
    ReadDistortionParams("distortion.txt",matDist);
    std::cout << "Distortion Coeff:\n" << matDist << "\n" << std::endl;

    cv::Mat img;

    //試しに先に進む
    Capture.set(cv::CAP_PROP_POS_FRAMES,1000);

    //動画の読み込み
    Capture >> img;
    cv::Size textureSize = cv::Size(2048,2048);
//    const int TEXTURE_W = 2048;//テクスチャ。TODO:ビデオのサイズに合わせて拡大縮小
//    const int TEXTURE_H = 2048;
    cv::Mat buff(textureSize.height,textureSize.width,CV_8UC3);//テクスチャ用Matを準備
    img.copyTo(buff(cv::Rect(0,0,img.cols,img.rows)));




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

        // Open a window and create its OpenGL context
        window = glfwCreateWindow( 1280, 720, "Tutorial 0 - Keyboard and Mouse", NULL, NULL);
//        window = glfwCreateWindow( 1920, 1080, "Tutorial 0 - Keyboard and Mouse", NULL, NULL);
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

        // Ensure we can capture the escape key being pressed below
        glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
        // Hide the mouse and enable unlimited mouvement
//        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        // Set the mouse at the center of the screen
        glfwPollEvents();
        glfwSetCursorPos(window, 1024/2, 768/2);

        // Dark blue background
        glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

        // Enable depth test
        glEnable(GL_DEPTH_TEST);
        // Accept fragment if it closer to the camera than the former one
        glDepthFunc(GL_LESS);

        // Cull triangles which normal is not towards the camera
        glEnable(GL_CULL_FACE);

        GLuint VertexArrayID;
        glGenVertexArrays(1, &VertexArrayID);
        glBindVertexArray(VertexArrayID);

        // Create and compile our GLSL program from the shaders
        GLuint programID = LoadShaders( "../biCubic/TransformVertexShader.vertexshader", "../biCubic/TextureFragmentShader.fragmentshader" );

        // Get a handle for our "MVP" uniform
        GLuint MatrixID = glGetUniformLocation(programID, "MVP");

        //////////////////
        ///
        // Create one OpenGL texture
        GLuint textureID_0;
        glGenTextures(1, &textureID_0);
        //OpenGLに「これから、テクスチャ識別子idに対して指示を与えます」と指示
        glBindTexture(GL_TEXTURE_2D,textureID_0);
        //テクスチャをここで作成
        glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,buff.cols,buff.rows,0,GL_BGR,GL_UNSIGNED_BYTE,buff.data);
        //////////////////

        static const GLfloat border[] = { 0.0, 1.0, 0.0, 1.0 };//背景色
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);//テクスチャの境界色
        //テクスチャの繰り返しの設定
//        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
//        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glGenerateMipmap(GL_TEXTURE_2D);
        // Load the texture
//        GLuint Texture = loadDDS("uvtemplate.DDS");

        // Get a handle for our "myTextureSampler" uniform
        GLuint TextureID  = glGetUniformLocation(programID, "myTextureSampler");

        // Our vertices. Tree consecutive floats give a 3D vertex; Three consecutive vertices give a triangle.
        // A cube has 6 faces with 2 triangles each, so this makes 6*2=12 triangles, and 12*3 vertices
        static const GLfloat g_vertex_buffer_data[] = {
            -1.0f,  1.0f, 0.0f,
            -1.0f, -1.0f, 0.0f,
             1.0f, 1.0f, 0.0f,


            -1.0f, -1.0f, 0.0f,
            1.0f,  -1.0f, 0.0f,
             1.0f, 1.0f, 0.0f,

        };

        // Two UV coordinatesfor each vertex. They were created with Blender.
        static const GLfloat g_uv_buffer_data[] = {
            0.0f, (float)imageSize.height/(float)textureSize.height,
            0.0f, 0.0f,
            (float)imageSize.width/(float)textureSize.width, (float)imageSize.height/(float)textureSize.height,


            0.0f, 0.0f,
            (float)imageSize.width/(float)textureSize.width, 0.0f,
            (float)imageSize.width/(float)textureSize.width, (float)imageSize.height/(float)textureSize.height,

        };
        //        static const GLfloat g_uv_buffer_data[] = {
//            0.0f, 1.0f,
//            0.0f, 0.0f,
//            1.0f, 1.0f,


//            0.0f, 0.0f,
//            1.0f, 0.0f,
//            1.0f, 1.0f,

//        };

        GLuint vertexbuffer;
        glGenBuffers(1, &vertexbuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

        GLuint uvbuffer;
        glGenBuffers(1, &uvbuffer);
        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data), g_uv_buffer_data, GL_STATIC_DRAW);

        //歪補正の準備
        GLuint nFxyID       = glGetUniformLocation(programID, "normalizedFocalLength");
        GLuint nCxyID       = glGetUniformLocation(programID, "normalizedOpticalCenter");
        GLuint distCoeffID  = glGetUniformLocation(programID, "distortionCoeffs");
//        float nfxy[] = {(float)(matIntrinsic.at<double>(0,0)/imageSize.width), (float)(matIntrinsic.at<double>(1,1)/imageSize.height)};
//        glUniform2fv(nFxyID, 1, nfxy);
//        float ncxy[] = {(float)(matIntrinsic.at<double>(0,2)/imageSize.width), (float)(matIntrinsic.at<double>(1,2)/imageSize.height)};
//        glUniform2fv(nCxyID, 1, ncxy);
//        float distcoeffFloat[] = {(float)(matIntrinsic.at<double>(0,0)),(float)(matIntrinsic.at<double>(0,1)),(float)(matIntrinsic.at<double>(0,2)),(float)(matIntrinsic.at<double>(0,3))};
//        glUniform4fv(distCoeffID, 1, distcoeffFloat);

        //テクスチャ座標の準備
        int32_t division_x = 5; //画面の横の分割数
        int32_t division_y = 5; //画面の縦の分割数
        std::vector<GLfloat> vecTexture;
        for(int j=0;j<division_y;++j){							//jは終了の判定が"<"であることに注意
            double v	= (double)j/division_y*imageSize.height;
            double v1	= (double)(j+1)/division_y*imageSize.height;
            for(int i=0;i<division_x;++i){
                double u	= (double)i/division_x*imageSize.width;
                double u1	= (double)(i+1)/division_x*imageSize.width;
                //OpenGL側へ送信するテクスチャの頂点座標を準備
                vecTexture.push_back((GLfloat)u/imageSize.width);//x座標
                vecTexture.push_back((GLfloat)v/imageSize.height);//y座標
                vecTexture.push_back((GLfloat)u/imageSize.width);//x座標
                vecTexture.push_back((GLfloat)v1/imageSize.height);//y座標
//                vecTexture.push_back((GLfloat)u1/TEXTURE_W);//x座標
//                vecTexture.push_back((GLfloat)v1/TEXTURE_H);//y座標
                vecTexture.push_back((GLfloat)u1/imageSize.width);//x座標
                vecTexture.push_back((GLfloat)v/imageSize.height);//y座標

//                vecTexture.push_back((GLfloat)u/TEXTURE_W);//x座標
//                vecTexture.push_back((GLfloat)v/TEXTURE_H);//y座標
                vecTexture.push_back((GLfloat)u/imageSize.width);//x座標
                vecTexture.push_back((GLfloat)v1/imageSize.height);//y座標
                vecTexture.push_back((GLfloat)u1/imageSize.width);//x座標
                vecTexture.push_back((GLfloat)v1/imageSize.height);//y座標
                vecTexture.push_back((GLfloat)u1/imageSize.width);//x座標
                vecTexture.push_back((GLfloat)v/imageSize.height);//y座標
            }
        }


        do{

            // Clear the screen
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // Use our shader
            glUseProgram(programID);

            // Compute the MVP matrix from keyboard and mouse input
//            computeMatricesFromInputs();
//            glm::mat4 ProjectionMatrix = getProjectionMatrix();
//            glm::mat4 ViewMatrix = getViewMatrix();
//            glm::mat4 ModelMatrix = glm::mat4(1.0);
//            glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
//            glm::mat4 MVP = glm::mat4(1.0f);//動画保存用
            glm::mat4 MVP = glm::rotate<float>(glm::mat4x4(),(float)M_PI,glm::vec3(0.0f,0.0f,1.0f));//画面表示用
            // Send our transformation to the currently bound shader,
            // in the "MVP" uniform
            glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

            // Bind our texture in Texture Unit 0
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, textureID_0);//            glBindTexture(GL_TEXTURE_2D, Texture);
            // Set our "myTextureSampler" sampler to user Texture Unit 0
            glUniform1i(TextureID, 0);

            //歪補正の準備
            float nfxy[] = {(float)(matIntrinsic.at<double>(0,0)/imageSize.width), (float)(matIntrinsic.at<double>(1,1)/imageSize.height)};
            glUniform2fv(nFxyID, 1, nfxy);
            float ncxy[] = {(float)(matIntrinsic.at<double>(0,2)/imageSize.width), (float)(matIntrinsic.at<double>(1,2)/imageSize.height)};
            glUniform2fv(nCxyID, 1, ncxy);
            float distcoeffFloat[] = {(float)(matDist.at<double>(0,0)),(float)(matDist.at<double>(0,1)),(float)(matDist.at<double>(0,2)),(float)(matDist.at<double>(0,3))};
            glUniform4fv(distCoeffID, 1, distcoeffFloat);

            // 1rst attribute buffer : vertices
            glEnableVertexAttribArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
            glVertexAttribPointer(
                0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
                3,                  // size
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
            glDrawArrays(GL_TRIANGLES, 0, 12*3); // 12*3 indices starting at 0 -> 12 triangles

            glDisableVertexAttribArray(0);
            glDisableVertexAttribArray(1);

            // Swap buffers
            glfwSwapBuffers(window);
            glfwPollEvents();

        } // Check if the ESC key was pressed or the window was closed
        while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
               glfwWindowShouldClose(window) == 0 );

        // Cleanup VBO and shader
        glDeleteBuffers(1, &vertexbuffer);
        glDeleteBuffers(1, &uvbuffer);
        glDeleteProgram(programID);
        glDeleteTextures(1, &TextureID);
        glDeleteVertexArrays(1, &VertexArrayID);
//ここでTextureID_0をDeleteしなくて大丈夫？
        // Close OpenGL window and terminate GLFW
        glfwTerminate();




    return 0;
}

