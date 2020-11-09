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
#include "json_tools.hpp"

using namespace std;
using namespace rapidjson;
#define SYNC_LENGTH 1000

using namespace Eigen;

CameraInformationJsonParser::CameraInformationJsonParser()
{
}

CameraInformationJsonParser::CameraInformationJsonParser(const char *camera_name, const char *lens_name, const char *image_size, const char *file_name)
{
    struct stat st;
    if (stat(file_name, &st))
    {
        throw "File doesn's exist";
    }
    FILE *fp = fopen(file_name, "rb"); // non-Windows use "r"
    std::vector<char> readBuffer((intmax_t)st.st_size + 10);
    rapidjson::FileReadStream is(fp, readBuffer.data(), readBuffer.size());
    Document e;
    e.ParseStream(is);
    fclose(fp);

    if (!e.HasMember(camera_name))
    {
        throw "Camera not found.";
    }

    const Value &camera = e[camera_name];
    sd_card_rotation_.w() = camera["quaternion_w"].GetDouble();
    sd_card_rotation_.x() = camera["quaternion_x"].GetDouble();
    sd_card_rotation_.y() = camera["quaternion_y"].GetDouble();
    sd_card_rotation_.z() = camera["quaternion_z"].GetDouble();

    camera_name_ = camera_name;
    if(!camera["lenses"].HasMember(lens_name)){
        throw "lense not found";
    }else if(!camera["lenses"][lens_name].HasMember(image_size)){
        throw "image size not found";
    }
    const Value &parameters = camera["lenses"][lens_name][image_size];
    lens_name_ = lens_name;
    fx_ = parameters["fx"].GetDouble();
    fy_ = parameters["fy"].GetDouble();
    cx_ = parameters["cx"].GetDouble();
    cy_ = parameters["cy"].GetDouble();
    k1_ = parameters["k1"].GetDouble();
    k2_ = parameters["k2"].GetDouble();
    p1_ = parameters["p1"].GetDouble();
    p2_ = parameters["p2"].GetDouble();
    line_delay_ = parameters["line_delay"].GetDouble();
    std::shared_ptr<char> ptr_image_size(new char[std::strlen(image_size)], std::default_delete<char[]>());
    strcpy(ptr_image_size.get(), image_size);
    width_ = std::atoi(strtok(ptr_image_size.get(), "x"));
    height_ = std::atoi(strtok(NULL, "x"));
}

void CameraInformationJsonParser::writeCameraInformationJson(const char *file_name)
{
    // If record json file exist, Open it.
    Document d;
    struct stat st;
    if (!stat(file_name, &st))
    {
        FILE *fp = fopen(file_name, "rb"); // non-Windows use "r"
        std::vector<char> readBuffer((intmax_t)st.st_size + 10);
        rapidjson::FileReadStream is(fp, readBuffer.data(), readBuffer.size());

        d.ParseStream(is);
        fclose(fp);
    }
    else
    {
        d = Document(kObjectType);
    }
    // Create camera json
    //   Document (kObjectType);
    Document camera(kObjectType);
    Value quaternion_w(sd_card_rotation_.w());
    Value quaternion_x(sd_card_rotation_.x());
    Value quaternion_y(sd_card_rotation_.y());
    Value quaternion_z(sd_card_rotation_.z());
    Document lenses(kObjectType);
    Document lens_name(kObjectType);
    Document image_size(kObjectType);

    Value fx(fx_);
    Value fy(fy_);
    Value cx(cx_);
    Value cy(cy_);
    Value k1(k1_);
    Value k2(k2_);
    Value p1(p1_);
    Value p2(p2_);
    Value line_delay(line_delay_);

    image_size.AddMember("fx", fx, image_size.GetAllocator());
    image_size.AddMember("fy", fy, image_size.GetAllocator());
    image_size.AddMember("cx", cx, image_size.GetAllocator());
    image_size.AddMember("cy", cy, image_size.GetAllocator());
    image_size.AddMember("k1", k1, image_size.GetAllocator());
    image_size.AddMember("k2", k2, image_size.GetAllocator());
    image_size.AddMember("p1", p1, image_size.GetAllocator());
    image_size.AddMember("p2", p2, image_size.GetAllocator());
    image_size.AddMember("line_delay", line_delay, image_size.GetAllocator());
    
    std::string image_size_string = std::to_string(width_) + std::string("x") + std::to_string(height_);
    
    // Same camera
    Value v_camera_name = Value(camera_name_.c_str(), d.GetAllocator());
    Value v_lens_name = Value(lens_name_.c_str(), d.GetAllocator());
    Value v_image_size = Value(image_size_string.c_str(), d.GetAllocator());
    if (d.HasMember(v_camera_name))
    {
        // d.RemoveMember(Value(camera_name_.c_str(),d.GetAllocator()));
        // Same Lens
        if (d[camera_name_.c_str()]["lenses"].HasMember(v_lens_name))
        {
            // Same size
            if (d[camera_name_.c_str()]["lenses"][lens_name_.c_str()].HasMember(v_image_size))
            {
                d[camera_name_.c_str()]["lenses"][lens_name_.c_str()].RemoveMember(v_image_size);
                d[camera_name_.c_str()]["lenses"][lens_name_.c_str()].AddMember(v_image_size, image_size, d.GetAllocator());
            }
            else
            {
                d[camera_name_.c_str()]["lenses"][lens_name_.c_str()].AddMember(v_image_size, image_size, d.GetAllocator());
            }
        }
        else
        {
            lens_name.AddMember(Value(image_size_string.c_str(), lens_name.GetAllocator()), image_size, lens_name.GetAllocator());
            d[camera_name_.c_str()]["lenses"].AddMember(v_lens_name, lens_name, d.GetAllocator());
        }
    }
    else
    {
       
        lens_name.AddMember(Value(image_size_string.c_str(), lens_name.GetAllocator()), image_size, lens_name.GetAllocator());
        lenses.AddMember(Value(lens_name_.c_str(), lenses.GetAllocator()), lens_name, lenses.GetAllocator());
        camera.AddMember("quaternion_w", quaternion_w, camera.GetAllocator());
        camera.AddMember("quaternion_x", quaternion_x, camera.GetAllocator());
        camera.AddMember("quaternion_y", quaternion_y, camera.GetAllocator());
        camera.AddMember("quaternion_z", quaternion_z, camera.GetAllocator());
        camera.AddMember("lenses", lenses, camera.GetAllocator());
        d.AddMember(v_camera_name, camera, d.GetAllocator());
    }

    FILE *fp = fopen(file_name, "wb"); // non-Windows use "w"

    char writeBuffer[262140];
    FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
    PrettyWriter<FileWriteStream> writer(os);
    d.Accept(writer);
    fclose(fp);
    return;
}

// bool readCameraInformation()
// {
// }

bool syncronizedQuarternionExist(const std::string &video_name)
{
    std::string json_file_name = videoNameToJsonName(video_name) + std::string(".sq");
    struct stat st;
    if (stat(json_file_name.c_str(), &st))
    {
        return false;
    }

    FILE *fp = fopen(json_file_name.c_str(), "rb"); // non-Windows use "r"
    char readBuffer[262140];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document e;
    e.ParseStream(is);
    fclose(fp);

    if (e.HasMember("synchronized_quaternion") && e.HasMember("synchronized_filtered_quaternion"))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void writeSynchronizedQuaternion(const Eigen::MatrixXd &raw_quaternion, const Eigen::MatrixXd &filtered_quaternion, const std::string video_name)
{
    Document d(kObjectType);

    Document v(kArrayType);

    Document::AllocatorType &allocator = v.GetAllocator();

    for (int i = 0, e = raw_quaternion.rows(); i < e; ++i)
    {
        v.PushBack(raw_quaternion(i, 0), allocator);
        v.PushBack(raw_quaternion(i, 1), allocator);
        v.PushBack(raw_quaternion(i, 2), allocator);
        v.PushBack(raw_quaternion(i, 3), allocator);
    }

    d.AddMember("synchronized_quaternion", v, d.GetAllocator());

    Document s(kArrayType);

    Document::AllocatorType &allocator_s = s.GetAllocator();

    for (int i = 0, e = filtered_quaternion.rows(); i < e; ++i)
    {
        s.PushBack(filtered_quaternion(i, 0), allocator_s);
        s.PushBack(filtered_quaternion(i, 1), allocator_s);
        s.PushBack(filtered_quaternion(i, 2), allocator_s);
        s.PushBack(filtered_quaternion(i, 3), allocator_s);
    }

    d.AddMember("synchronized_filtered_quaternion", s, d.GetAllocator());

    std::string json_file_name = videoNameToJsonName(video_name) + std::string(".sq");

    FILE *fp = fopen(json_file_name.c_str(), "wb"); // non-Windows use "w"

    char writeBuffer[262140];
    FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
    Writer<FileWriteStream> writer(os);
    d.Accept(writer);
    fclose(fp);
    return;
}

int readSynchronizedQuaternion(Eigen::MatrixXd &raw_quaternion, Eigen::MatrixXd &filtered_quaternion, const std::string video_name)
{
    FILE *fp = fopen((videoNameToJsonName(video_name) + std::string(".sq")).c_str(), "rb"); // non-Windows use "r"
    char readBuffer[262140];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document e;
    e.ParseStream(is);
    fclose(fp);

    const Value &raw_quaternion_array = e["synchronized_quaternion"];
    int width = 4;
    raw_quaternion.resize(raw_quaternion_array.Size() / width, width);
    for (int r = 0; r < raw_quaternion.rows(); ++r)
    {
        for (int c = 0; c < width; ++c)
        {
            raw_quaternion(r, c) = raw_quaternion_array[r * width + c].GetDouble();
        }
    }

    const Value &filtered_quaternion_array = e["synchronized_filtered_quaternion"];
    //    int width = 4;
    filtered_quaternion.resize(filtered_quaternion_array.Size() / width, width);
    for (int r = 0; r < filtered_quaternion.rows(); ++r)
    {
        for (int c = 0; c < width; ++c)
        {
            filtered_quaternion(r, c) = filtered_quaternion_array[r * width + c].GetDouble();
        }
    }

    return 0;
}

std::string videoNameToJsonName(std::string video_name)
{
    std::string json_file_name = video_name;
    std::string::size_type pos;
    if ((pos = json_file_name.find_last_of(".")) != std::string::npos)
    {
        //拡張子の長さをチェック
        if ((json_file_name.length() - json_file_name.substr(0, pos).length()) > 4)
        {
            json_file_name += std::string(".json");
        }
        else
        {
            //拡張子の判定が3文字以下なら拡張子を削除して.jsonをつける
            json_file_name = json_file_name.substr(0, pos) + std::string(".json");
        }
    }
    return json_file_name;
}

double readSamplingRateFromJson(const char* filename){
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

    return e["frequency"].GetDouble();
    
}

bool jsonExists(std::string video_file_name)
{
    std::string json_file_name = videoNameToJsonName(video_file_name);
    struct stat st;
    return !stat(json_file_name.c_str(), &st);
}

int writeOpticalFrowToJson(std::string video_file_name, Eigen::MatrixXd &optical_flow, Eigen::MatrixXd &confidence)
{

    Document d(kObjectType);

    Document v(kArrayType);
    Document::AllocatorType &allocator = v.GetAllocator();
    for (int i = 0, e = optical_flow.rows(); i < e; ++i)
    {
        v.PushBack(optical_flow(i, 0), allocator);
        v.PushBack(optical_flow(i, 1), allocator);
        v.PushBack(optical_flow(i, 2), allocator);
    }
    d.AddMember("optical_flow", v, d.GetAllocator());

    Document c(kArrayType);
    Document::AllocatorType &allocator_c = c.GetAllocator();
    for (int i = 0, e = confidence.rows(); i < e; ++i)
    {
        c.PushBack(confidence(i, 0), allocator_c);
    }
    d.AddMember("confidence", c, d.GetAllocator());

    std::string json_file_name = videoNameToJsonName(video_file_name);

    FILE *fp = fopen(json_file_name.c_str(), "wb"); // non-Windows use "w"

    char writeBuffer[262140];
    FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
    Writer<FileWriteStream> writer(os);
    d.Accept(writer);
    fclose(fp);
    return 0;
}

int readOpticalFlowFromJson(std::string video_file_name, Eigen::MatrixXd &optical_flow, Eigen::MatrixXd &confidence)
{

    FILE *fp = fopen(videoNameToJsonName(video_file_name).c_str(), "rb"); // non-Windows use "r"
    char readBuffer[262140];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document e;
    e.ParseStream(is);
    fclose(fp);

    const Value &optical_flow_array = e["optical_flow"];
    optical_flow.resize(optical_flow_array.Size() / 3, 3);
    for (int r = 0; r < optical_flow.rows(); ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            optical_flow(r, c) = optical_flow_array[r * 3 + c].GetDouble();
        }
    }

    const Value &confidence_array = e["confidence"];
    confidence.resize(confidence_array.Size() , 1);
    for (int r = 0; r < confidence.rows(); ++r)
    {
            confidence(r) = confidence_array[r].GetDouble();
    }

    return 0;
}


Eigen::MatrixXd readAngularVelocityFromJson(const char* filename){
    struct stat st;
    Eigen::MatrixXd retval;
    if(stat(filename,&st)){
        throw "Json file is not exists.";
    }
    FILE* fp = fopen(filename, "rb"); // non-Windows use "r"
    std::vector<char> readBuffer((intmax_t)st.st_size+10);
    rapidjson::FileReadStream is(fp, readBuffer.data(), readBuffer.size());
    rapidjson::Document e;
    e.ParseStream(is);
    fclose(fp);

    const rapidjson::Value& angular_velocity_rad_per_sec_array = e["angular_velocity_rad_per_sec"];
    assert(angular_velocity_rad_per_sec_array.IsArray());
    assert(angular_velocity_rad_per_sec_array[0][0].IsDouble() || angular_velocity_rad_per_sec_array[0][0].IsInt());
    int32_t total_number_of_data=0;
    for(size_t num=0;num<angular_velocity_rad_per_sec_array.Size();++num){
        total_number_of_data += angular_velocity_rad_per_sec_array[num].Size();
    }

    int width = 3;
    retval.resize(total_number_of_data/width,3);
    int num=0;
    for(size_t record =0;record<angular_velocity_rad_per_sec_array.Size();++record){
        for(size_t i=0;i<angular_velocity_rad_per_sec_array[record].Size();++i){
            retval(num/3,num%3)=angular_velocity_rad_per_sec_array[record][i].GetDouble();
            ++num;
        }
    }
    return retval;
}
//int main(int argc, char** argv){
//    int opt;

//    char *videoPass = NULL;
//    char *csvPass = NULL;
//    while((opt = getopt(argc, argv, "i:c:")) != -1){
//        switch (opt) {
//        case 'i':       //input video file pass
//            videoPass = optarg;
//            break;
//        case 'c':       //input angular velocity csv file pass
//            csvPass = optarg;
//            break;
//        default :
//            return 1;
//        }
//    }

//    writeAngularVelocityJson(std::string(videoPass));

//    return 0;
//}
