#ifndef CAMERAINFORMATION_H
#define CAMERAINFORMATION_H

#include <stdio.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>
class CameraInformation{
public:
    CameraInformation();
    CameraInformation(std::string camera_name,std::string lens_name,Eigen::Quaterniond sd_card_rotation,int32_t width,int32_t height,
                      double fx,double fy,double cx,double cy,double k1,double k2,double p1,
                      double p2,double rolling_shutter_coefficient);


    std::string camera_name_;
    std::string lens_name_;
    Eigen::Quaterniond sd_card_rotation_;
    int32_t width_;
    int32_t height_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double k1_;
    double k2_;
    double p1_;
    double p2_;
    double inverse_k1_;
    double inverse_k2_;
    double inverse_p1_;
    double inverse_p2_;
    double rolling_shutter_coefficient_;
};

#endif // CAMERAINFORMATION_H
