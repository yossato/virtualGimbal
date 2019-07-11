#include <stdio.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <memory>
#include "virtual_gimbal_manager.h"
#include "json_tools.hpp"
#include "rotation_param.h"
#include "distortion.h"
#include <sys/types.h>
#include <sys/stat.h>

// #define __DEBUG_ONLY
#ifdef __DEBUG_ONLY
#include "visualizer.h"

#endif
using namespace std;

int main(int argc, char **argv)
{
    //引数の確認
    char *videoPass = NULL;
    char *cameraName = NULL;
    char *lensName = NULL;
    char *jsonPass = NULL;
    const char *kernel_name = "cl/stabilizer_kernel.cl";
    const char *kernel_function = "stabilizer_function";
    bool debug_speedup = false;
    double zoom = 1.0;
    int opt;
    //    Eigen::Quaterniond camera_rotation;

    while ((opt = getopt(argc, argv, "j:i:c:l:d::z:k::f::")) != -1)
    {
        switch (opt)
        {
        case 'j': //input json file from virtual gimbal
            jsonPass = optarg;
            break;
        case 'i': //input video file pass
            videoPass = optarg;
            break;
        case 'c':
            cameraName = optarg;
            break;
        case 'l':
            lensName = optarg;
            break;
        case 'd':
            debug_speedup = true;
            break;
        case 'z':       //zoom ratio, dafault 1.0
            zoom = std::stof(optarg);
            break;
        case 'k':
            kernel_name = optarg;
            break;
        case 'f':
            kernel_function = optarg;
            break;
        default:
            //            printf(     "virtualGimbal\r\n"
            //                        "Hyper fast video stabilizer\r\n\r\n"
            //                        "usage: virtualGimbal [-i video] [-f angularVelocity] [[output option] -o] [options]\r\n"
            //                        );
            return 1;
        }
    }

    struct stat st;
    if (stat(kernel_name, &st))
    {
        std::cerr << "Kernel file not found." << std::endl << std::flush;
        throw "Kernel file not found.";
    }


    VirtualGimbalManager manager;
    manager.kernel_function = kernel_function;
    manager.kernel_name = kernel_name;

    // TODO:Check kernel availability here. Build once.

    shared_ptr<CameraInformation> camera_info(new CameraInformationJsonParser(cameraName, lensName, VirtualGimbalManager::getVideoSize(videoPass).c_str()));
    calcInverseDistortCoeff(*camera_info);
    manager.setMeasuredAngularVelocity(jsonPass, camera_info);
    manager.setVideoParam(videoPass, camera_info);
    Eigen::MatrixXd estimated_angular_velocity,confidence;
    if(debug_speedup){
        manager.estimateAngularVelocity(estimated_angular_velocity,confidence,100);
    }else{
        manager.estimateAngularVelocity(estimated_angular_velocity,confidence,1000);
    }
    
    manager.setEstimatedAngularVelocity(estimated_angular_velocity, confidence);
    // std::cout << "estimated_angular_velocity:" << estimated_angular_velocity.transpose() << std::endl
    //           << std::flush;
    // std::cout << "confidence:" << confidence.transpose() << std::endl
    //           << std::flush;
              

    Eigen::VectorXd correlation = manager.getCorrelationCoefficient();
    double offset = manager.getSubframeOffset(correlation);
    manager.setResamplerParameter(offset);
#ifdef __DEBUG_ONLY
    std::vector<string> legends_angular_velocity = {"c"};
    vgp::plot(correlation, "correlation", legends_angular_velocity);
#endif 
    // manager.getTiming()
    // manager.setTiming()
    auto fir_filter = std::make_shared<KaiserWindowFilter>(199,2);
    manager.setFilter(fir_filter);
    manager.setMaximumGradient(0.5);
    Eigen::VectorXd filter_coefficients = manager.getFilterCoefficients(zoom,*fir_filter,2,499);
#ifdef __DEBUG_ONLY
    vgp::plot(filter_coefficients, "filter_coefficients", legends_angular_velocity);
#endif
    // manager.getFilteredRotation();
    // manager.spin()



    manager.spin(zoom,*fir_filter,filter_coefficients);

    return 0;
}