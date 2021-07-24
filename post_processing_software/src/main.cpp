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
    // Check arguments
    char *videoPass = NULL;
    char *cameraName = NULL;
    char *lensName = NULL;
    char *jsonPass = NULL;
    bool output = false;
    bool show_image = true;
    bool inpainting = false;
    const char *kernel_name = "cl/stabilizer_kernel.cl";
    const char *kernel_function = "stabilizer_function";
    double zoom = 1.0;
    int32_t fileter_length = 199;
    int opt;
    int queue_size = 10;
    size_t buffer_size = 21;
    while ((opt = getopt(argc, argv, "j:i:c:l:w:z:k:f:o::n::p::b:")) != -1)
    {
        switch (opt)
        {
        case 'j': //input json file from virtual gimbal
            jsonPass = optarg;
            break;
        case 'i': //input video file pass
            videoPass = optarg;
            printf("videoPass %s\r\n",videoPass);
            break;
        case 'c':
            cameraName = optarg;
            break;
        case 'l':
            lensName = optarg;
            break;
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
        // case 'p':
        //     inpainting = true;
        //     break;
        // case 'b':
        //     buffer_size = std::stoi(optarg);
        //     break;
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
        std::cerr << "ERROR: Kernel file not found. " << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
        std::exit(EXIT_FAILURE);
    }

    if (stat(videoPass, &st))
    {
        std::cerr << "ERROR: " << videoPass << " is not found. "  << std::endl << std::flush;
        std::exit(EXIT_FAILURE) ;
    }


    VirtualGimbalManager manager(queue_size);
    manager.kernel_function = kernel_function;
    manager.kernel_name = kernel_name;

    // TODO:Check kernel availability here. Build once.

    shared_ptr<CameraInformation> camera_info;
    try
    {
        camera_info = shared_ptr<CameraInformation>( new CameraInformationJsonParser(cameraName, lensName, VirtualGimbalManager::getVideoSize(videoPass).c_str()));        
    }
    catch(std::string e)
    {
        std::cerr << "Error: " << e <<  std::endl;
        std::exit(EXIT_FAILURE);
    }
    calcInverseDistortCoeff(*camera_info);
    manager.setMeasuredAngularVelocity(jsonPass, camera_info);
    manager.setVideoParam(videoPass, camera_info);

    if(output){
        try
        {
            manager.enableWriter(videoPass);
        }
        catch(const char& e)
        {
            std::cerr << "Error: " << e << std::endl;
            std::exit(EXIT_FAILURE);
        }
        
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

    auto table = manager.getSyncTable(30.0,999);
    if(2 >table.size()){
        printf("Warning: Input video too short to apply poly line syncronize method, an alternative mothod is used.\r\n");
        table = manager.getSyncTableOfShortVideo();
    }
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

    if(inpainting)
    {
        manager.spinInpainting(zoom,table,fir_filter,buffer_size);
    }
    else
    {
        manager.spin(zoom,fir_filter,filter_coefficients,table, show_image);    
    }
    

    return 0;
}