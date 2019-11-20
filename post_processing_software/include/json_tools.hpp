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
int writeOpticalFrowToJson(std::string video_file_name, Eigen::MatrixXd &optical_flow, Eigen::MatrixXd &confidence);
int readOpticalFlowFromJson(std::string video_file_name, Eigen::MatrixXd &optical_flow, Eigen::MatrixXd &confidence);
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

    // const rapidjson::Value& ff = ;
    assert(e["frequency"].IsDouble());
    const rapidjson::Value& angular_velocity_rad_per_sec_array = e["angular_velocity_rad_per_sec"];
    assert(angular_velocity_rad_per_sec_array.IsArray());
    assert(angular_velocity_rad_per_sec_array[0][0].IsDouble());
    _Tp val;
    int width = 3;
    for(size_t a =0;a<angular_velocity_rad_per_sec_array.Size();++a){
        for(size_t i=0;i<angular_velocity_rad_per_sec_array[a].Size();i+=width){
            for(int k=0;k<width;++k){
                val[k]=angular_velocity_rad_per_sec_array[a][i+k].GetDouble();
            }
            angular_velocity.push_back(val);
        }
    }
    return 0;
}

Eigen::MatrixXd readAngularVelocityFromJson(const char* filename);

#endif // JSON_TOOLS_HPP
