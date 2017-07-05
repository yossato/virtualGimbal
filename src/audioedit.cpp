#include <stdio.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <unistd.h>
int main(int argc, char** argv){

    //引数の確認
    char *inputVideoPass = NULL;
    int opt;
    while((opt = getopt(argc, argv, "i:")) != -1){
        switch (opt) {
        case 'i':
            inputVideoPass = optarg;
            break;
        default :
            printf("Use options. -i input video filepass -a filepass of input video with audio.\r\n");
            return 1;
        }
    }

    if(inputVideoPass==NULL){
        printf("Use options. -i input video filepass.\r\n");
        return 1;
    }

    //inputVideoPassにスペースがあったらエスケープシーケンスを追加
    std::string::size_type pos = 0;
   std::string sInputVideoPass = inputVideoPass;
    while(pos=sInputVideoPass.find(" ",pos), pos!=std::string::npos){
        sInputVideoPass.insert(pos,"\\");
        pos+=2;
    }
ipadからの変更テスト

    std::cout << "音声を分離" << std::endl;
    std::string command = "ffmpeg -i " + sInputVideoPass +  " -vn -acodec copy output-audio.aac";
    system(command.c_str());
    std::cout << "音声を結合" << std::endl;
    command = "ffmpeg -i " + sInputVideoPass + "_deblured.avi -i output-audio.aac -codec copy " + sInputVideoPass + "_deblured_audio.avi";
    system(command.c_str());

    system("rm output-audio.aac");
    command = "rm " + sInputVideoPass + "_deblured.avi";
    system(command.c_str());
    return 0;
}
