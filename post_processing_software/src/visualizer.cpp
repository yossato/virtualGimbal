/*************************************************************************
*  Software License Agreement (BSD 3-Clause License)
*  
*  Copyright (c) 2019, Yoshiaki Sato
*  All rights reserved.
*  
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*  
*  1. Redistributions of source code must retain the above copyright notice, this
*     list of conditions and the following disclaimer.
*  
*  2. Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*  
*  3. Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived from
*     this software without specific prior written permission.
*  
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*************************************************************************/
#include "visualizer.h"
#include "matplotlib-cpp/matplotlibcpp.h"
#include "rotation_math.h"
namespace plt = matplotlibcpp;
using namespace std;
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
    for(size_t i=0;i<index.size();i++){
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

void plot(vector<Eigen::Quaterniond> data, string title, string legend_x, string legend_y, string legend_z){
    vector<double> x,y,z,index;
    //Refill
    for(auto el:data){
        Eigen::Vector3d vec = Quaternion2Vector(el);
        x.push_back(vec[0]);
        y.push_back(vec[1]);
        z.push_back(vec[2]);
    }
    index.resize(x.size());
    for(size_t i=0;i<index.size();i++){
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

void plot(vector<double> x,vector<double>  y,vector<double> z, string title, string legend_x, string legend_y, string legend_z){
    vector<double> index;
    index.resize(x.size());
    for(size_t i=0;i<index.size();i++){
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

void plot(const Eigen::MatrixXd mat, std::string title, std::vector<std::string> &legends){
    vector<double> index(mat.rows());
    vector<string> format = {"r","g","b","m","y","c"};
    for(size_t i=0;i<index.size();i++){
        index[i] = static_cast<double>(i);
    }
    vector<double> value;
    for(size_t i=0,e=mat.cols();i<e;++i){
        value.resize(mat.rows());
        Eigen::MatrixXd part = mat.block(0,i,mat.rows(),1);
        memcpy(value.data(),part.data(),mat.rows()*sizeof(double));

        // if(i < legends.size()){
//            plt::named_plot(legends[i],index,value);
            plt::plot(index,value,format[format.size()>i?i:format.size()-1]);
        // }else{
            // plt::plot(index,value);
        // }

    }

    plt::legend();//enable legend
    if(!title.empty()){
        plt::title(title.c_str());
    }
    plt::show();
}

//void plot(Eigen::VectorXd x, Eigen::VectorXd y, std::string style){
//    plt::plot(x,y,style.c_str());
//}

}
