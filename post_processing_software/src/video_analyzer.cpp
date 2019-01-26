#include <stdio.h>
#include <string>
#include <unistd.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include "calcShift.hpp"
using namespace std;

#define SYNC_LENGTH 1000

int main(int argc, char** argv){
    int opt;

    char *videoPass = NULL;
    char *csvPass = NULL;
    while((opt = getopt(argc, argv, "i:c:")) != -1){
        string value1 ;//= optarg;
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
    

    return 0;
}