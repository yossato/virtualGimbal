#include "video_analyzer.hpp"

using namespace std;
using namespace rapidjson;
#define SYNC_LENGTH 1000

using namespace Eigen;


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
