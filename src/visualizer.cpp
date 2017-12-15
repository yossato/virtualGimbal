#include "visualizer.h"
#include "stabilize.h"
#include "matplotlib-cpp/matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace std;
using namespace boost::math;
namespace vgp
{
/**
 * @brief plot vectorized cv::Vec3d
 * @param data
 * @param title
 * @param legend_x
 * @param legend_y
 * @param legend_z
 */
void plot(vector<cv::Vec3d> data, string title, string legend_x, string legend_y, string legend_z){
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
    if(legend_x.empty()){
        plt::named_plot("x",index,x);
    }else{
        plt::named_plot(legend_x.c_str(),index,x);
    }
    if(legend_y.empty()){
        plt::named_plot("y",index,y);
    }else{
        plt::named_plot(legend_y.c_str(),index,y);
    }

    if(legend_z.empty()){
        plt::named_plot("z",index,z);
    }else{
        plt::named_plot(legend_z.c_str(),index,z);
    }
    plt::legend();//enable legend
    if(!title.empty()){
        plt::title(title.c_str());
    }
    plt::show();
}

void plot(vector<quaternion<double>> data, string title, string legend_x, string legend_y, string legend_z){
    vector<double> x,y,z,index;
    //Refill
    for(auto el:data){
        cv::Vec3d vec = Quaternion2Vector(el);
        x.push_back(vec[0]);
        y.push_back(vec[1]);
        z.push_back(vec[2]);
    }
    index.resize(x.size());
    for(int i=0;i<index.size();i++){
        index[i] = static_cast<double>(i);
    }
    if(legend_x.empty()){
        plt::named_plot("x",index,x);
    }else{
        plt::named_plot(legend_x.c_str(),index,x);
    }
    if(legend_y.empty()){
        plt::named_plot("y",index,y);
    }else{
        plt::named_plot(legend_y.c_str(),index,y);
    }

    if(legend_z.empty()){
        plt::named_plot("z",index,z);
    }else{
        plt::named_plot(legend_z.c_str(),index,z);
    }
    plt::legend();//enable legend
    if(!title.empty()){
        plt::title(title.c_str());
    }
    plt::show();
}
}
