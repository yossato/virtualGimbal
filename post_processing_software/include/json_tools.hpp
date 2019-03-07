    #ifndef JSON_TOOLS_HPP
#define JSON_TOOLS_HPP

#include <stdio.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>
#include <Eigen/Dense>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filewritestream.h"
#include "rapidjson/filereadstream.h"
#include "memory"
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>

#include "camera_information.h"



class CameraInformationJsonParser : public CameraInformation
{
public:
    CameraInformationJsonParser();
    CameraInformationJsonParser(const char* camera_name, const char* lens_name, const char* image_size, const char* file_name="camera_descriptions/cameras.json");
    void writeCameraInformationJson(const char* file_name="camera_descriptions/cameras.json");
};

bool syncronizedQuarternionExist(const std::string &video_name);
void writeSynchronizedQuaternion(const Eigen::MatrixXd &raw_quaternion, const Eigen::MatrixXd &filtered_quaternion, const std::string video_name);
int readSynchronizedQuaternion( Eigen::MatrixXd &raw_quaternion, Eigen::MatrixXd &filtered_quaternion, const std::string video_name);
bool jsonExists(std::string video_file_name);
int writeOpticalFrowToJson(Eigen::MatrixXd &optical_flow,std::string video_file_name);
int readOpticalFlowFromJson(Eigen::MatrixXd &optical_flow,std::string video_file_name);
std::string videoNameToJsonName(std::string video_name);



double readSamplingRateFromJson(const char* filename);


template <typename _Tp, typename _Alloc = std::allocator<_Tp>> int readAngularVelocityFromJson(std::vector<_Tp,_Alloc> &angular_velocity, const char* filename){
    struct stat st;
    if(stat(filename,&st)){
        return -1;
    }
    FILE* fp = fopen(filename, "rb"); // non-Windows use "r"
    std::vector<char> readBuffer((intmax_t)st.st_size+10);
    rapidjson::FileReadStream is(fp, readBuffer.data(), readBuffer.size());
    rapidjson::Document e;
    e.ParseStream(is);
    fclose(fp);

    const rapidjson::Value& ff = e["frequency"];
    assert(ff.IsDouble());
    const rapidjson::Value& angular_velocity_rad_per_sec_array = e["angular_velocity_rad_per_sec"];
    assert(angular_velocity_rad_per_sec_array.IsArray());
    assert(angular_velocity_rad_per_sec_array[0][0].IsDouble());
    _Tp val;
    int width = 3;
    for(int a =0;a<angular_velocity_rad_per_sec_array.Size();++a){
        for(int i=0;i<angular_velocity_rad_per_sec_array[a].Size();i+=width){
            for(int k=0;k<width;++k){
                val[k]=angular_velocity_rad_per_sec_array[a][i+k].GetDouble();
            }
            angular_velocity.push_back(val);
        }
    }
    return 0;
}

#endif // JSON_TOOLS_HPP
