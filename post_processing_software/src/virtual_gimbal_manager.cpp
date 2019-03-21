#include "virtual_gimbal_manager.h"

VirtualGimbalManager::VirtualGimbalManager()
{

}

void VirtualGimbalManager::setVideoParam(const char *file_name){
    cv::VideoCapture *Capture = new cv::VideoCapture(file_name);//動画をオープン
    video_param.reset(new Video(Capture->get(cv::CAP_PROP_FPS), file_name));
    video_param->video_frames = Capture->get(cv::CAP_PROP_FRAME_COUNT);
    video_param->rolling_shutter_time = 0.0;
}
