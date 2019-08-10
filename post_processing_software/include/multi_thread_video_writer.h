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
#include "Eigen/Dense"
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

template <typename TYPE>
class MultiThreadQueue
{
public:
    MultiThreadQueue() : max_size_(100){};

    int push(TYPE &p)
    {
        while (1)
        {
            size_t data_size;
            {
                std::lock_guard<std::mutex> lock(mutex);
                data_size = data.size();
            }
            if (data_size < max_size_)
            {
                break;
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                // std::cout << "Waiting, data.size:" << data.size() << std::endl;
            }
        }

        std::lock_guard<std::mutex> lock(mutex);
        data.emplace(std::move(p));
        return 0;
    }

    int get(TYPE &p)
    {
        std::lock_guard<std::mutex> lock(mutex);
        p = std::move(data.front());
        return 0;
    }

    int pop()
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (data.empty())
        {
            return 1;
        }
        data.pop();
        // std::cout << "data size:" << data.size() << std::endl;
        return 0;
    }

    bool empty()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return !data.size();
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex);
        while (!data.empty())
        {
            data.pop();
        }
    }
    // int size();
private:
    std::queue<TYPE> data;
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
    MultiThreadQueue<UMatPtr> write_data_;
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
    MultiThreadQueue<UMatPtr> read_data_;
    volatile bool is_reading;
    cv::VideoCapture video_capture;
    std::thread th1;
    void join();
    void videoReaderProcess();
};

using MatrixPtr = std::unique_ptr<std::vector<float>>;
class MultiThreadRotationMatrixGenerator
{
public:
    MultiThreadRotationMatrixGenerator(VideoPtr video_parameter,
                                       ResamplerParameterPtr resampler_parameter,
                                       KaiserWindowFilter filter,
                                       AngularVelocityPtr measured_angular_velocity,
                                       Eigen::VectorXd filter_strength);
    int get(MatrixPtr &p);
    ~MultiThreadRotationMatrixGenerator();

private:
    MultiThreadQueue<MatrixPtr> rotation_matrix_;
    int32_t rows;
    VideoPtr video_parameter;
    ResamplerParameterPtr resampler_parameter;
    KaiserWindowFilter filter;
    AngularVelocityPtr measured_angular_velocity;
    Eigen::VectorXd filter_strength;
    volatile bool is_reading;
    std::thread th1;
    void join();
    void process();
};

#endif //__MULTI_THREAD_VIDEO_WRITER_H__