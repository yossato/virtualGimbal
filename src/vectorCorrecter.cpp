#include <stdio.h>
#include <iostream>
#include "matplotlib-cpp/matplotlibcpp.h"
#include <opencv2/opencv.hpp>
namespace plt = matplotlibcpp;
using namespace std;

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

    //upper
    for(int x=0;x<=division_x;++x){
        contour_x.push_back((double)x/division_x*imageSize.width);
        contour_y.push_back(0);
    }
    //middle
    for(int y=1;y<division_y;++y){
        for(int x=0;x<=division_x;x+=division_x){
            contour_x.push_back((double)x/division_x*imageSize.width);
            contour_y.push_back((double)y/division_y*imageSize.height);
        }
    }
    //bottom
    for(int x=0;x<=division_x;++x){
        contour_x.push_back((double)x/division_x*imageSize.width);
        contour_y.push_back(imageSize.height);
    }
//cout << "size" << contour_x.size() << endl;
    plt::plot(contour_x,contour_y, "xr");
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
