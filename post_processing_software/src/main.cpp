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
    bool output = false;
    bool show_image = true;
    const char *kernel_name = "cl/stabilizer_kernel.cl";
    const char *kernel_function = "stabilizer_function";
    // bool debug_speedup = false;
    double zoom = 1.0;
    int32_t fileter_length = 199;
    int opt;
    int queue_size = 10;
    //    Eigen::Quaterniond camera_rotation;

    while ((opt = getopt(argc, argv, "j:i:c:l:w:z:k:f:o::n::")) != -1)
    {
        switch (opt)
        {
        case 'j': //input json file from virtual gimbal
            jsonPass = optarg;
            break;
        case 'i': //input video file pass
            videoPass = optarg;
            printf("videoPass %s",videoPass);
            break;
        case 'c':
            cameraName = optarg;
            break;
        case 'l':
            lensName = optarg;
            break;
        // case 'd':
        //     debug_speedup = true;
        //     break;
        case 'z':       //zoom ratio, dafault 1.0
            zoom = std::stof(optarg);
            break;
        case 'w':
            fileter_length = std::stoi(optarg);
            break;
        case 'q':
            queue_size = std::stoi(optarg);
            break;
        case 'k':
            kernel_name = optarg;
            break;
        case 'f':
            kernel_function = optarg;
            break;
        case 'o':
            output = true;
            break;
        case 'n':
            show_image = false;
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


    VirtualGimbalManager manager(queue_size);
    manager.kernel_function = kernel_function;
    manager.kernel_name = kernel_name;

    // TODO:Check kernel availability here. Build once.

    shared_ptr<CameraInformation> camera_info(new CameraInformationJsonParser(cameraName, lensName, VirtualGimbalManager::getVideoSize(videoPass).c_str()));
    calcInverseDistortCoeff(*camera_info);
    manager.setMeasuredAngularVelocity(jsonPass, camera_info);
    manager.setVideoParam(videoPass, camera_info);

    if(output){
        manager.enableWriter(videoPass);
    }


    

    Eigen::MatrixXd estimated_angular_velocity,confidence;
    manager.estimateAngularVelocity(estimated_angular_velocity,confidence);
    
    manager.setEstimatedAngularVelocity(estimated_angular_velocity, confidence);
    // std::cout << "confidence:" << confidence.transpose() << std::endl
    //           << std::flush;
              

    // Eigen::VectorXd correlation = manager.getCorrelationCoefficient(0,1000);
    // double offset = manager.getSubframeOffsetInSecond(correlation,0,1000);
    // manager.setResamplerParameter(offset);

    // Eigen::VectorXd correlation_begin,correlation_end;
    // auto modified_resampler_parameter = manager.getResamplerParameterWithClockError(correlation_begin,correlation_end);
    // manager.setResamplerParameter(modified_resampler_parameter);
// #ifdef __DEBUG_ONLY
//     std::vector<string> legends_angular_velocity = {"c"};
//     vgp::plot(correlation_begin, "correlation_begin", legends_angular_velocity);
//     vgp::plot(correlation_end, "correlation_end", legends_angular_velocity);
// #endif 





// #ifdef __DEBUG_ONLY
    // std::vector<string> legends_angular_velocity = {"c"};
//     vgp::plot(correlation, "correlation", legends_angular_velocity);
// #endif 

    auto fir_filter = std::make_shared<NormalDistributionFilter>();
    manager.setFilter(fir_filter);
    manager.setMaximumGradient(0.5);

    auto table = manager.getSyncTable(30*24,999);
    printf("Table:\r\n");
    for(size_t i=0;i<table.size()-1;++i)
    {
        printf("(%d,%f), a:%f b=%f\r\n",table[i].first,table[i].second,
        (table[i+1].second-table[i].second)/(table[i+1].first-table[i].first),
        (table[i].second*table[i+1].first-table[i].first*table[i+1].second)/(table[i+1].first-table[i].first)
        );
    }


    Eigen::VectorXd filter_coefficients = manager.getFilterCoefficients(zoom,fir_filter,table,fileter_length,0); // Zero is the weakest value since apply no filter, output is equal to input.
#ifdef __DEBUG_ONLY
    std::vector<string> legends_angular_velocity = {"c"};
    vgp::plot(filter_coefficients, "filter_coefficients", legends_angular_velocity);
#endif

    manager.spin(zoom,fir_filter,filter_coefficients,table, show_image);

    return 0;
}