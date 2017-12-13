#ifndef VISUALIZER_H
#define VISUALIZER_H
#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>
#include "matplotlib-cpp/matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace std;

void show(vector<cv::Vec3d> data, string title = "", string legend_x = "", string legend_y = "", string legend_z = ""){
    vector<double> x,y,z,index;
    //Refill
    for(auto el:data){
        x.push_back(el[0]);
        y.push_back(el[1]);
        z.push_back(el[2]);
    }
    index.resize(x.size());
    for(int i=0;i<index.size();i++){
        index[i] = static_cast<double>(i);
    }
    plt::named_plot("x",index,x);
    plt::named_plot("y",index,y);
    plt::named_plot("z",index,z);
    plt::legend();//enable legend
    if(!title.empty()){
        plt::title(title.c_str());
    }
    plt::show();
}

#endif // VISUALIZER_H
