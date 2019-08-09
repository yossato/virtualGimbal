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
#include <deque>

#include <opencv2/opencv.hpp>
#include <memory>

using UMatPtr = std::unique_ptr<cv::UMat>;

// class UMatWithMutex
// {
// public:
//     UMatWithMutex(UMatPtr &p);
// public:
//     UMatPtr ptr;
//     std::mutex mutex;
// };

// using UMatWithMutexPtr = std::unique_ptr<UMatWithMutex>;

class MultiThreadVideoData
{
public:
    MultiThreadVideoData() : max_size_(100){};
    int push(UMatPtr &p);
    int get(UMatPtr &p);
    int pop();
    bool empty();
    void clear();
    // int size();
private:
    std::queue<UMatPtr> data;
    std::mutex mutex;
    size_t max_size_;
};


class MultiThreadVideoWriter
{
public:
    MultiThreadVideoWriter(std::string output_pass, Video &video_param);
    ~MultiThreadVideoWriter();
    std::string output_name(char *source_name);
    static std::string getOutputName(const char *source_video_name);
    int push(UMatPtr &p);
private:
    MultiThreadVideoData write_data_;
    volatile bool is_writing;
    cv::VideoWriter video_writer;
    std::thread th1;
    void join();
    void videoWriterProcess();
};

class MultiThreadVideoReader
{
public:
    MultiThreadVideoReader(std::string input_path);
    ~MultiThreadVideoReader();
    int get(UMatPtr &p);
private:
    MultiThreadVideoData read_data_;
    volatile bool is_reading;
    cv::VideoCapture video_capture;
    std::thread th1;
    void join();
    void videoReaderProcess();

};

#endif //__MULTI_THREAD_VIDEO_WRITER_H__