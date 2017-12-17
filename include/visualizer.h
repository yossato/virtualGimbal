#ifndef VISUALIZER_H
#define VISUALIZER_H
#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>
//#include "matplotlib-cpp/matplotlibcpp.h"
#include <boost/math/quaternion.hpp>
//#include "stabilize.h"
#include <Eigen/Dense>
namespace vgp
{

    void plot(std::vector<cv::Vec3d> data, std::string title = "", std::string legend_x = "", std::string legend_y = "", std::string legend_z = "");
    void plot(std::vector<boost::math::quaternion<double>> data, std::string title = "", std::string legend_x = "", std::string legend_y = "", std::string legend_z = "");
    void plot(std::vector<double> x, std::vector<double>  y,std::vector<double> z, std::string title = "", std::string legend_x = "", std::string legend_y = "", std::string legend_z = "");
    void plot(const Eigen::MatrixXd mat, std::string title = "",  std::vector<std::string> legends = std::vector<std::string>());
}

#endif // VISUALIZER_H
