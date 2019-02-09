#include "camera_information.h"

CameraInformation::CameraInformation():inverse_k1_(0.),inverse_k2_(0.),inverse_p1_(0.),inverse_p2_(0.){}
CameraInformation::CameraInformation(std::string camera_name, std::string lens_name, Eigen::Quaterniond sd_card_rotation, int32_t width, int32_t height, double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2, double rolling_shutter_coefficient) :
    camera_name_(camera_name),lens_name_(lens_name),sd_card_rotation_(sd_card_rotation),
    width_(width),height_(height),fx_(fx),fy_(fy),cx_(cx),cy_(cy),k1_(k1),k2_(k2),p1_(p1),p2_(p2),rolling_shutter_coefficient_(rolling_shutter_coefficient),
    inverse_k1_(0.),inverse_k2_(0.),inverse_p1_(0.),inverse_p2_(0.){

}
