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

    std::cout << "音声を分離" << std::endl;
    std::string command = "ffmpeg -i " + (std::string)inputVideoPass +  " -vn -acodec copy output-audio.aac";
    system(command.c_str());
    std::cout << "音声を結合" << std::endl;
    command = "ffmpeg -i " + (std::string)inputVideoPass + "_deblured.avi -i output-audio.aac -codec copy " + (std::string)inputVideoPass + "_deblured_audio.avi";
    system(command.c_str());
    system("rm output-audio.aac");
    return 0;
}
