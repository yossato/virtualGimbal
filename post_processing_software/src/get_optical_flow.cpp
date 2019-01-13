#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "settings.h"
#include "calcShift.hpp"

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filewritestream.h"

using namespace rapidjson;
int main(int argc, char** argv){
    char *videoPass = NULL;
    int opt;
    while((opt = getopt(argc, argv, "i:")) != -1){
        std::string value1 ;//= optarg;
        switch (opt) {
        case 'i':       //input video file pass
            videoPass = optarg;
            break;
        default :
            printf(     "virtualGimbal\r\n"
                        "Hyper fast video stabilizer\r\n\r\n"
                        "usage: virtualGimbal [-i video] [-f angularVelocity] [[output option] -o] [options]\r\n"
                        );
            return 1;
        }
    }
    std::vector<cv::Vec3d> opticShift = CalcShiftFromVideo(videoPass,SYNC_LENGTH);//ビデオからオプティカルフローを用いてシフト量を算出
    
    Document d(kObjectType);

    Document v(kArrayType);
    Document::AllocatorType& allocator = v.GetAllocator();
    for(auto el:opticShift){
        v.PushBack(el[0],allocator);
        v.PushBack(el[1],allocator);
        v.PushBack(el[2],allocator);
    }

    d.AddMember("optical_flow",v,d.GetAllocator());

    FILE* fp = fopen((std::string(videoPass) + std::string(".json")).c_str(), "wb"); // non-Windows use "w"

    char writeBuffer[65536];
    FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
    Writer<FileWriteStream> writer(os);
    d.Accept(writer);
    fclose(fp);
    return 0;
}
