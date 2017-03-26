#include <stdio.h>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

// Include standard headers
#include <stdio.h>
#include <stdlib.h>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include <shader.hpp>
#include <texture.hpp>
#include <controls.hpp>


/**
* @brief カメラ行列設定を記録したテキストファイルを読み込みます
* @param [in] FileName		テキストファイルの名前。例:"intrinsic.txt"
* @param [out] Values		読み取った値を書き込む行列。
* @retval 0で正常
**/
int ReadIntrinsicsParams(const char* FileName, cv::Mat &Values){
    std::ifstream ifs(FileName);//CSVファイルを開く
    if(!ifs){
        printf("エラー：カメラ行列を記録した%sが見つかりません\nプログラムを終了します\n",FileName);//TODO:abotrせずに開けなかったことを上に返すようにして、その場合は推定値を使うようにする
        abort();
    }
    std::string str;
    Values = cv::Mat::zeros(3,3,CV_64F);//matを初期化

    int j=0;
    while(getline(ifs,str)){//1行ずつ読み込む
        std::string token;
        std::istringstream stream(str);

        int i=0;
        while(getline(stream, token, ' ')){
            Values.at<double>(j,i++) = (double)stof(token);//値を保存
            if(i>3){
                printf("エラー:カメラ行列のパラメータファイルintrinsic.txtの書式が不正なため、プログラムを終了します。\n");
                abort();
            }
        }
        ++j;
    }

    return 0;

}


/**
* @brief 歪補正パラメータを記録したテキストファイルを読み込みます
* @param [in] FileName		テキストファイルの名前。例:"distortion.txt"
* @param [out] Values		読み取った値を書き込む構造体。
* @retval 0で正常
**/
//template<typename T_dcoef>
int ReadDistortionParams(const char FileName[], cv::Mat &Values){
    std::ifstream ifs(FileName);//CSVファイルを開く
    if(!ifs){
        printf("エラー：歪パラメータを記録した%sが見つかりません\nプログラムを終了します\n",FileName);//TODO:abotrせずに開けなかったことを上に返すようにして、その場合は推定値を使うようにする
        abort();
    }
    std::string str;
    Values = cv::Mat::zeros(1,4,CV_64F);//matを初期化

    int j=0;
    while(getline(ifs,str)){//1行ずつ読み込む
        std::string token;
        std::istringstream stream(str);

        int i=0;
        while(getline(stream, token, ' ')){
            Values.at<double>(0,i++) = (double)stof(token);//値を保存
        }
    }
    return 0;



    //エラーチェック
    assert(Values.cols == 5);
    assert(Values.rows == 1);

    FILE *fp;
    const int Num = 5;
    // テキストファイルを読み込み用に開く
    if (fopen(FileName, "r") != 0){
        // ファイルオープンに失敗
        printf("File Open Error. \"%s\" is not found.\n", FileName);
        return 1;
    }

    double valuesd[5] = { 0.0 };
    if (fscanf(fp, "%lf %lf %lf %lf %lf", &valuesd[0], &valuesd[1], &valuesd[2], &valuesd[3], &valuesd[4]) == -1){//1行読み込み
        //エラー処理
        printf("エラー:歪パラメータファイルの読み込みに失敗しました。\r\n");
        printf("Check format of \"%s\".\n", FileName);
        fclose(fp);
        return 1;
    }
    int i;
    printf("%s\r\n", FileName);
    for (i = 0; i < Num; i++){
        //printf("%f%t\n", valuesd[i]);
        Values.at<double>(0, i) = valuesd[i];//cv::Matに代入
    }
    std::cout << Values << std::endl;
    printf("\r\n");
    fclose(fp);
    return 0;

}

int main(int argc, char** argv){
    cv::VideoCapture Capture(argv[1]);//動画をオープン
    assert(Capture.isOpened());
    cv::Size resolution = cv::Size(Capture.get(CV_CAP_PROP_FRAME_WIDTH),Capture.get(CV_CAP_PROP_FRAME_HEIGHT));//解像度を読む
    double samplingPeriod = 1.0/Capture.get(CV_CAP_PROP_FPS);
    std::cout << "resolution" << resolution << std::endl;
    std::cout << "samplingPeriod" << samplingPeriod << std::endl;

    //レンズ歪データを読み込み
    cv::Mat matIntrinsic;
    ReadIntrinsicsParams("intrinsic.txt",matIntrinsic);
    std::cout << "Camera matrix:\n" << matIntrinsic << "\n" <<  std::endl;

    //歪パラメータの読み込み
    cv::Mat matDist;
    ReadDistortionParams("distortion.txt",matDist);
    std::cout << "Distortion Coeff:\n" << matDist << "\n" << std::endl;



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
        window = glfwCreateWindow( 1024, 768, "Tutorial 0 - Keyboard and Mouse", NULL, NULL);
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
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

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
        GLuint programID = LoadShaders( "TransformVertexShader.vertexshader", "TextureFragmentShader.fragmentshader" );

        // Get a handle for our "MVP" uniform
        GLuint MatrixID = glGetUniformLocation(programID, "MVP");

        // Load the texture
        GLuint Texture = loadDDS("uvtemplate.DDS");

        // Get a handle for our "myTextureSampler" uniform
        GLuint TextureID  = glGetUniformLocation(programID, "myTextureSampler");

        // Our vertices. Tree consecutive floats give a 3D vertex; Three consecutive vertices give a triangle.
        // A cube has 6 faces with 2 triangles each, so this makes 6*2=12 triangles, and 12*3 vertices
        static const GLfloat g_vertex_buffer_data[] = {
            -1.0f,-1.0f,-1.0f,
            -1.0f,-1.0f, 1.0f,
            -1.0f, 1.0f, 1.0f,
             1.0f, 1.0f,-1.0f,
            -1.0f,-1.0f,-1.0f,
            -1.0f, 1.0f,-1.0f,
             1.0f,-1.0f, 1.0f,
            -1.0f,-1.0f,-1.0f,
             1.0f,-1.0f,-1.0f,
             1.0f, 1.0f,-1.0f,
             1.0f,-1.0f,-1.0f,
            -1.0f,-1.0f,-1.0f,
            -1.0f,-1.0f,-1.0f,
            -1.0f, 1.0f, 1.0f,
            -1.0f, 1.0f,-1.0f,
             1.0f,-1.0f, 1.0f,
            -1.0f,-1.0f, 1.0f,
            -1.0f,-1.0f,-1.0f,
            -1.0f, 1.0f, 1.0f,
            -1.0f,-1.0f, 1.0f,
             1.0f,-1.0f, 1.0f,
             1.0f, 1.0f, 1.0f,
             1.0f,-1.0f,-1.0f,
             1.0f, 1.0f,-1.0f,
             1.0f,-1.0f,-1.0f,
             1.0f, 1.0f, 1.0f,
             1.0f,-1.0f, 1.0f,
             1.0f, 1.0f, 1.0f,
             1.0f, 1.0f,-1.0f,
            -1.0f, 1.0f,-1.0f,
             1.0f, 1.0f, 1.0f,
            -1.0f, 1.0f,-1.0f,
            -1.0f, 1.0f, 1.0f,
             1.0f, 1.0f, 1.0f,
            -1.0f, 1.0f, 1.0f,
             1.0f,-1.0f, 1.0f
        };

        // Two UV coordinatesfor each vertex. They were created with Blender.
        static const GLfloat g_uv_buffer_data[] = {
            0.000059f, 0.000004f,
            0.000103f, 0.336048f,
            0.335973f, 0.335903f,
            1.000023f, 0.000013f,
            0.667979f, 0.335851f,
            0.999958f, 0.336064f,
            0.667979f, 0.335851f,
            0.336024f, 0.671877f,
            0.667969f, 0.671889f,
            1.000023f, 0.000013f,
            0.668104f, 0.000013f,
            0.667979f, 0.335851f,
            0.000059f, 0.000004f,
            0.335973f, 0.335903f,
            0.336098f, 0.000071f,
            0.667979f, 0.335851f,
            0.335973f, 0.335903f,
            0.336024f, 0.671877f,
            1.000004f, 0.671847f,
            0.999958f, 0.336064f,
            0.667979f, 0.335851f,
            0.668104f, 0.000013f,
            0.335973f, 0.335903f,
            0.667979f, 0.335851f,
            0.335973f, 0.335903f,
            0.668104f, 0.000013f,
            0.336098f, 0.000071f,
            0.000103f, 0.336048f,
            0.000004f, 0.671870f,
            0.336024f, 0.671877f,
            0.000103f, 0.336048f,
            0.336024f, 0.671877f,
            0.335973f, 0.335903f,
            0.667969f, 0.671889f,
            1.000004f, 0.671847f,
            0.667979f, 0.335851f
        };

        GLuint vertexbuffer;
        glGenBuffers(1, &vertexbuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

        GLuint uvbuffer;
        glGenBuffers(1, &uvbuffer);
        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data), g_uv_buffer_data, GL_STATIC_DRAW);

        do{

            // Clear the screen
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // Use our shader
            glUseProgram(programID);

            // Compute the MVP matrix from keyboard and mouse input
            computeMatricesFromInputs();
            glm::mat4 ProjectionMatrix = getProjectionMatrix();
            glm::mat4 ViewMatrix = getViewMatrix();
            glm::mat4 ModelMatrix = glm::mat4(1.0);
            glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

            // Send our transformation to the currently bound shader,
            // in the "MVP" uniform
            glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

            // Bind our texture in Texture Unit 0
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, Texture);
            // Set our "myTextureSampler" sampler to user Texture Unit 0
            glUniform1i(TextureID, 0);

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

        // Close OpenGL window and terminate GLFW
        glfwTerminate();




    return 0;
}
