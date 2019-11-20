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
    MultiThreadQueue(size_t queue_size) : max_size_(queue_size){};

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
    MultiThreadVideoWriter(std::string output_pass, Video &video_param, size_t queue_size);
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
    MultiThreadVideoReader(std::string input_path,size_t queue_size);
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
                                    //    ResamplerParameterPtr resampler_parameter,
                                       FilterPtr filter,
                                       AngularVelocityPtr measured_angular_velocity,
                                       Eigen::VectorXd filter_strength,
                                       std::vector<std::pair<int32_t,double>> sync_table,
                                       size_t queue_size);
    int get(MatrixPtr &p);
    ~MultiThreadRotationMatrixGenerator();

private:

    int32_t rows;
    VideoPtr video_parameter;
    // ResamplerParameterPtr resampler_parameter;
    FilterPtr filter;
    AngularVelocityPtr measured_angular_velocity;
    Eigen::VectorXd filter_strength;
    std::vector<std::pair<int32_t,double>> sync_table;
    MultiThreadQueue<MatrixPtr> rotation_matrix_;
    volatile bool is_reading;
    std::thread th1;
    void join();
    void process();
};

#endif //__MULTI_THREAD_VIDEO_WRITER_H__