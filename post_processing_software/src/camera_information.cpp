#include "camera_information.h"

CameraInformation::CameraInformation():camera_name_(""),lens_name_(""),sd_card_rotation_(Eigen::Quaterniond()),
    width_(0),height_(0),fx_(0.),fy_(0.),cx_(0.),cy_(0.),k1_(0.),k2_(0.),p1_(0.),p2_(0.),line_delay_(0.),
    inverse_k1_(0.),inverse_k2_(0.),inverse_p1_(0.),inverse_p2_(0.)
    {}
CameraInformation::CameraInformation(std::string camera_name, std::string lens_name, Eigen::Quaterniond sd_card_rotation, int32_t width, int32_t height, double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2, double line_delay) :
    camera_name_(camera_name),lens_name_(lens_name),sd_card_rotation_(sd_card_rotation),
    width_(width),height_(height),fx_(fx),fy_(fy),cx_(cx),cy_(cy),k1_(k1),k2_(k2),p1_(p1),p2_(p2),line_delay_(line_delay),
    inverse_k1_(0.),inverse_k2_(0.),inverse_p1_(0.),inverse_p2_(0.)
{

}
