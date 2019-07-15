#ifndef __MULTI_THREAD_VIDEO_WRITER_H__
#define __MULTI_THREAD_VIDEO_WRITER_H__

#include <memory>
#include <mutex>
#include <thread>
#include <deque>
#include <string>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include "rotation_param.h"
#include <opencv2/opencv.hpp>

class MultiThreadVideoWriter
{
public:
    MultiThreadVideoWriter(std::string output_pass, Video &video_param);
    
    // void beginThread();
    void addFrame(cv::Mat image);
    ~MultiThreadVideoWriter();
    std::string output_name(char *source_name);
    static std::string getOutputName(const char *source_video_name);
private:
    std::mutex mtx;
    volatile bool is_writing;
    std::deque<cv::Mat> images;
    cv::VideoWriter video_writer;
    std::thread th1;
    void join();
    void videoWriterProcess();
};

#endif //__MULTI_THREAD_VIDEO_WRITER_H__