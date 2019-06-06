#include <stdio.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <memory>
#include "virtual_gimbal_manager.h"
#include "json_tools.hpp"
#include "rotation_param.h"
#include "distortion.h"
using namespace std;

int main(int argc, char **argv){
    //引数の確認
    char *videoPass = NULL;
    char *cameraName = NULL;
    char *lensName = NULL;
    char *jsonPass = NULL;
    bool debug_speedup = false;
    int opt;
    //    Eigen::Quaterniond camera_rotation;

    while ((opt = getopt(argc, argv, "j:i:c:l:d::")) != -1)
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

        default:
            //            printf(     "virtualGimbal\r\n"
            //                        "Hyper fast video stabilizer\r\n\r\n"
            //                        "usage: virtualGimbal [-i video] [-f angularVelocity] [[output option] -o] [options]\r\n"
            //                        );
            return 1;
        }
    }

    VirtualGimbalManager manager;
    shared_ptr<CameraInformation> camera_info(new CameraInformationJsonParser(cameraName, lensName, VirtualGimbalManager::getVideoSize(videoPass).c_str()));
    calcInverseDistortCoeff(*camera_info);
    manager.setMeasuredAngularVelocity(jsonPass, camera_info);
    manager.setVideoParam(videoPass, camera_info);
    // manager.estimateAngularVelocity()    //オプティカルフローから角速度検出
    // manager.getCorrelation() //タイミング検出のために相関を計算
    // manager.getTiming()
    // manager.setTiming()
    // manager.spin()

}