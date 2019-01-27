#include <stdio.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include "calcShift.hpp"


#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filewritestream.h"

using namespace std;
using namespace rapidjson;
#define SYNC_LENGTH 1000

int main(int argc, char** argv){
    int opt;

    char *videoPass = NULL;
    char *csvPass = NULL;
    while((opt = getopt(argc, argv, "i:c:")) != -1){
        switch (opt) {
        case 'i':       //input video file pass
            videoPass = optarg;
            break;
        case 'c':       //input angular velocity csv file pass
            csvPass = optarg;
            break;
        default :
            return 1;
        }
    }

    //Extract Optical flow
    vector<cv::Vec3d> opticShift = CalcShiftFromVideo(videoPass,SYNC_LENGTH);//ビデオからオプティカルフローを用いてシフト量を算出
    
    for(auto el:opticShift){
        printf("%f %f %f\n",el[0],el[1],el[2]);
    }

    Document d(kObjectType);

    Document v(kArrayType);
    Document::AllocatorType& allocator = v.GetAllocator();
    for(auto el:opticShift){
        v.PushBack(el[0],allocator);
        v.PushBack(el[1],allocator);
        v.PushBack(el[2],allocator);
    }

    d.AddMember("optical_flow",v,d.GetAllocator());

    std::string json_file_name = std::string(videoPass);// + std::string(".json");
    std::string::size_type pos;
    if((pos = json_file_name.find_last_of(".")) != std::string::npos){
        //拡張子の長さをチェック
        if((json_file_name.length() - json_file_name.substr(0,pos).length())>4){
            json_file_name += std::string(".json");
        }else{
            //拡張子の判定が3文字以下なら拡張子を削除して.jsonをつける
            json_file_name = json_file_name.substr(0,pos) + std::string(".json");
        }
    }

    FILE* fp = fopen(json_file_name.c_str(), "wb"); // non-Windows use "w"

    char writeBuffer[65536];
    FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
    Writer<FileWriteStream> writer(os);
    d.Accept(writer);
    fclose(fp);

    return 0;
}
