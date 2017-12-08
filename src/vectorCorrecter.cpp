#include <stdio.h>
#include <iostream>
#include "matplotlib-cpp/matplotlibcpp.h"
#include <opencv2/opencv.hpp>
#include "stabilize.h"
#include "distortion.h"
#include "mINIRead.hpp"
namespace plt = matplotlibcpp;
using namespace std;

template <typename _Tp> bool check_warp(vector<_Tp> &contour){
    for(int i=0;i<contour.size();i+=2){
        if((abs(contour[i]) < 1.0)&&(abs(contour[i+1]) < 1.0)){
            return false;
        }
    }
    return true;
}

int main(int argc, char** argv){
    //define devisions
    int32_t division_x = 9; //horizontal
    int32_t division_y = 9; //virtical
    
    //define image size
    cv::Size imageSize;
    imageSize.width = 1920;
    imageSize.height = 1080;
    
    vector<double> contour_x;
    vector<double> contour_y;

    //    //upper
    //    for(int x=0;x<=division_x;++x){
    //        contour_x.push_back((double)x/division_x*imageSize.width);
    //        contour_y.push_back(0);
    //    }
    //    //middle
    //    for(int y=1;y<division_y;++y){
    //        for(int x=0;x<=division_x;x+=division_x){
    //            contour_x.push_back((double)x/division_x*imageSize.width);
    //            contour_y.push_back((double)y/division_y*imageSize.height);
    //        }
    //    }
    //    //bottom
    //    for(int x=0;x<=division_x;++x){
    //        contour_x.push_back((double)x/division_x*imageSize.width);
    //        contour_y.push_back(imageSize.height);
    //    }

    //    plt::plot(contour_x,contour_y, "xr");
    //    plt::show();


    //cout << prevDiffAngleQuaternion << endl;

    //内部パラメータを読み込み
    cv::Mat matIntrinsic;
    ReadIntrinsicsParams("intrinsic.txt",matIntrinsic);
    std::cout << "Camera matrix:\n" << matIntrinsic << "\n" <<  std::endl;
    double fx = matIntrinsic.at<double>(0,0);
    double fy = matIntrinsic.at<double>(1,1);
    double cx = matIntrinsic.at<double>(0,2);
    double cy = matIntrinsic.at<double>(1,2);

    //歪パラメータの読み込み
    cv::Mat matDist;
    ReadDistortionParams("distortion.txt",matDist);
    std::cout << "Distortion Coeff:\n" << matDist << "\n" << std::endl;

    //逆歪パラメータの計算
    cv::Mat matInvDistort;
    calcDistortCoeff(matIntrinsic,matDist,imageSize,matInvDistort);
    std::vector<float> vecVtx;
    quaternion<double> prevDiffAngleQuaternion(1,0,0,0);
    quaternion<double> currDiffAngleQuaternion(1,0,0,0);
    quaternion<double> nextDiffAngleQuaternion(1,0,0,0);
//    quaternion<double> adjustmentQuaternion(1,0,0,0);
    quaternion<double> adjustmentQuaternion = RotationQuaternion(cv::Vec3d(0,0,0.1));

    getDistortUnrollingContour(prevDiffAngleQuaternion,
                               currDiffAngleQuaternion,
                               nextDiffAngleQuaternion,
                               division_x,
                               division_y,
                               0.0,
                               matInvDistort,
                               matIntrinsic,
                               imageSize,
                               adjustmentQuaternion,
                               vecVtx,
                               1.0);

    for(auto el:vecVtx){
        cout << el << endl;
    }

    if(check_warp(vecVtx) == true){
        cout << "成功" << endl;
    }else{
        cout << "失敗" << endl;
    }

    auto func = [&vecVtx](string s){
        vector<double> retval;
        if(s == string("x")){
            for(int i=0;i<vecVtx.size();i+=2){
                retval.push_back(vecVtx[i]);
            }
        }else if(s == string("y")){
            for(int i=1;i<vecVtx.size();i+=2){
                retval.push_back(vecVtx[i]);
            }
        }else{
            abort();
        }
        return retval;
    };



    vector<double> vx,vy;
    vx = func("x");
    vy = func("y");
    plt::plot(vx, vy, "xr");

    vector<double> sx={-1,1,1,-1,-1};
    vector<double> sy={1,1,-1,-1,1};
    plt::plot(sx,sy);

    plt::show();

    return 0;

    //////////////////////////////////////
    plt::plot({1,2,3,4});
    plt::show();

    int n = 5000;
    vector<double> x(n), y(n), z(n);
    for(int i=0; i<n; ++i) {
        x.at(i) = i;
        y.at(i) = sin(2*M_PI*i/360.0);
        z.at(i) = cos(2*M_PI*i/360.0);
    }

    plt::plot(x, y, "xr");
    plt::show();

    std::cout << "Hello world." << std::endl;
}
