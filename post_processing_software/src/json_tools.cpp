#include "json_tools.hpp"

using namespace std;
using namespace rapidjson;
#define SYNC_LENGTH 1000

using namespace Eigen;

CameraInformationJsonParser::CameraInformationJsonParser(const char *camera_name, const char *lens_name, const char *image_size, const char *file_name){
    struct stat st;
    if( stat(file_name,&st)){
        throw "File doesn's exist";
    }
    FILE* fp = fopen(file_name, "rb"); // non-Windows use "r"
    std::vector<char> readBuffer((intmax_t)st.st_size+10);
    rapidjson::FileReadStream is(fp, readBuffer.data(), readBuffer.size());
    Document e;
    e.ParseStream(is);
    fclose(fp);

    if(!e.HasMember(camera_name)){
        throw "Camera not found.";
    }

    const Value& camera = e[camera_name];
    sd_card_rotation_.w() = camera["quaternion_w"].GetDouble();
    sd_card_rotation_.x() = camera["quaternion_x"].GetDouble();
    sd_card_rotation_.y() = camera["quaternion_y"].GetDouble();
    sd_card_rotation_.z() = camera["quaternion_z"].GetDouble();

    camera_name_ = camera_name;
    const Value& parameters = camera["lenses"][lens_name][image_size];
    lens_name_ = lens_name;
    fx_ = parameters["fx"].GetDouble();
    fy_ = parameters["fy"].GetDouble();
    cx_ = parameters["cx"].GetDouble();
    cy_ = parameters["cy"].GetDouble();
    k1_ = parameters["k1"].GetDouble();
    k2_ = parameters["k2"].GetDouble();
    p1_ = parameters["p1"].GetDouble();
    p2_ = parameters["p2"].GetDouble();
    rolling_shutter_coefficient_ = parameters["rolling_shutter_coefficient"].GetDouble();
    std::shared_ptr<char> ptr_image_size(new char[std::strlen(image_size)], std::default_delete<char[]>());
    strcpy(ptr_image_size.get(),image_size);
    width_ = std::atoi(strtok(ptr_image_size.get(),"x"));
    height_ = std::atoi(strtok(NULL,"x"));
}



bool readCameraInformation(){

}

bool syncronizedQuarternionExist(const std::string &video_name){
    std::string json_file_name = videoNameToJsonName(video_name) + std::string(".sq");
    struct stat st;
    if( stat(json_file_name.c_str(),&st)){
        return false;
    }

    FILE* fp = fopen(json_file_name.c_str(), "rb"); // non-Windows use "r"
    char readBuffer[262140];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document e;
    e.ParseStream(is);
    fclose(fp);

    if(e.HasMember("synchronized_quaternion") && e.HasMember("synchronized_filtered_quaternion")){
        return true;
    }else{
        return false;
    }


}

void writeSynchronizedQuaternion(const Eigen::MatrixXd &raw_quaternion, const Eigen::MatrixXd &filtered_quaternion, const std::string video_name){
    Document d(kObjectType);

    Document v(kArrayType);

    Document::AllocatorType& allocator = v.GetAllocator();

    for(int i=0,e=raw_quaternion.rows();i<e;++i){
        v.PushBack(raw_quaternion(i,0),allocator);
        v.PushBack(raw_quaternion(i,1),allocator);
        v.PushBack(raw_quaternion(i,2),allocator);
        v.PushBack(raw_quaternion(i,3),allocator);
    }

    d.AddMember("synchronized_quaternion",v,d.GetAllocator());

    Document s(kArrayType);

    Document::AllocatorType& allocator_s = s.GetAllocator();

    for(int i=0,e=filtered_quaternion.rows();i<e;++i){
        s.PushBack(filtered_quaternion(i,0),allocator_s);
        s.PushBack(filtered_quaternion(i,1),allocator_s);
        s.PushBack(filtered_quaternion(i,2),allocator_s);
        s.PushBack(filtered_quaternion(i,3),allocator_s);
    }

    d.AddMember("synchronized_filtered_quaternion",s,d.GetAllocator());


    std::string json_file_name = videoNameToJsonName(video_name) + std::string(".sq");

    FILE* fp = fopen(json_file_name.c_str(), "wb"); // non-Windows use "w"

    char writeBuffer[262140];
    FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
    Writer<FileWriteStream> writer(os);
    d.Accept(writer);
    fclose(fp);
    return ;
}

int readSynchronizedQuaternion( Eigen::MatrixXd &raw_quaternion, Eigen::MatrixXd &filtered_quaternion, const std::string video_name){
     FILE* fp = fopen((videoNameToJsonName(video_name) + std::string(".sq")).c_str(), "rb"); // non-Windows use "r"
    char readBuffer[262140];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document e;
    e.ParseStream(is);
    fclose(fp);

    const Value& raw_quaternion_array = e["synchronized_quaternion"];
    int width = 4;
    raw_quaternion.resize(raw_quaternion_array.Size()/width,width);
    for(int r=0;r<raw_quaternion.rows();++r){
        for(int c=0;c<width;++c){
            raw_quaternion(r,c)=raw_quaternion_array[r*width+c].GetDouble();
        }
    }

    const Value& filtered_quaternion_array = e["synchronized_filtered_quaternion"];
//    int width = 4;
    filtered_quaternion.resize(filtered_quaternion_array.Size()/width,width);
    for(int r=0;r<filtered_quaternion.rows();++r){
        for(int c=0;c<width;++c){
            filtered_quaternion(r,c)=filtered_quaternion_array[r*width+c].GetDouble();
        }
    }

    return 0;
}

std::string videoNameToJsonName(std::string video_name){
    std::string json_file_name = video_name;
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
    return json_file_name;
}

bool jsonExists(std::string video_file_name){
    std::string json_file_name = videoNameToJsonName(video_file_name);
    struct stat st;
    return !stat(json_file_name.c_str(),&st);
}


int writeOpticalFrowToJson(Eigen::MatrixXd &optical_flow, std::string video_file_name){

    Document d(kObjectType);

    Document v(kArrayType);

    Document::AllocatorType& allocator = v.GetAllocator();

    for(int i=0,e=optical_flow.rows();i<e;++i){
        v.PushBack(optical_flow(i,0),allocator);
        v.PushBack(optical_flow(i,1),allocator);
        v.PushBack(optical_flow(i,2),allocator);
    }

    d.AddMember("optical_flow",v,d.GetAllocator());

    std::string json_file_name = videoNameToJsonName(video_file_name);

    FILE* fp = fopen(json_file_name.c_str(), "wb"); // non-Windows use "w"

    char writeBuffer[262140];
    FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
    Writer<FileWriteStream> writer(os);
    d.Accept(writer);
    fclose(fp);
    return 0;
}

int readOpticalFlowFromJson(Eigen::MatrixXd &optical_flow,std::string video_file_name){


    FILE* fp = fopen(videoNameToJsonName(video_file_name).c_str(), "rb"); // non-Windows use "r"
    char readBuffer[262140];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document e;
    e.ParseStream(is);
    fclose(fp);

    const Value& optical_flow_array = e["optical_flow"];
    optical_flow.resize(optical_flow_array.Size()/3,3);
    for(int r=0;r<optical_flow.rows();++r){
        for(int c=0;c<3;++c){
            optical_flow(r,c)=optical_flow_array[r*3+c].GetDouble();
        }
    }

    return 0;
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
