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
#include "multi_thread_video_writer.h"

// UMatWithMutex::UMatWithMutex(UMatPtr &p){
//     ptr = std::move(p);
// }

// int MultiThreadData::size(){
//     std::lock_guard<std::mutex> lock(mutex);
//     return data.size();
// }

MultiThreadVideoWriter::MultiThreadVideoWriter(std::string output_pass, Video &video_param, size_t queue_size) : write_data_(queue_size)
{
    struct stat st;
    if (!stat(output_pass.c_str(), &st))
    {
        std::cerr << "Output file already exist." << std::endl;
        throw "Output file already exist.";
    }

    video_writer = cv::VideoWriter(output_pass, cv::VideoWriter::fourcc('F', 'M', 'P', '4'), video_param.getFrequency(), cv::Size(video_param.camera_info->width_, video_param.camera_info->height_), true);
    if (!video_writer.isOpened())
    {
        std::cerr << "Error: Can't Open Video Writer." << std::endl;
        throw "Error: Can't open video writer.";
    }

    is_writing = true;
    th1 = std::thread(&MultiThreadVideoWriter::videoWriterProcess, this); // Run thread
}

void MultiThreadVideoWriter::videoWriterProcess()
{
    while (1)
    {
        if (!write_data_.empty())
        {
            UMatPtr data_to_write;
            cv::UMat bgr;
            write_data_.get(data_to_write);
            cv::cvtColor(*data_to_write, bgr, cv::COLOR_BGRA2BGR);
            video_writer << bgr;
            write_data_.pop();
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if ((!is_writing) && write_data_.empty())
        {
            return;
        }
    }
}

std::string MultiThreadVideoWriter::getOutputName(const char *source_video_name)
{
    std::string output_pass(source_video_name);
    std::string::size_type pos;

    // Get day and time
    time_t now = time(nullptr);
    const tm *lt = localtime(&now);
    std::stringstream s;
    s << "20";
    s << lt->tm_year - 100;
    s << "-";
    s << lt->tm_mon + 1;
    s << "-";
    s << lt->tm_mday;
    s << ".";
    s << lt->tm_hour;
    s << "-";
    s << lt->tm_min;
    s << "-";
    s << lt->tm_sec;

    if ((pos = output_pass.find_last_of(".")) == std::string::npos)
    {
        output_pass = output_pass + "_stabilized_" + s.str() + ".avi";
    }
    else
    {
        output_pass.substr(0, pos);
        output_pass = output_pass.substr(0, pos) + "_stabilized_" + s.str() + ".avi";
    }
    return output_pass;
}

MultiThreadVideoWriter::~MultiThreadVideoWriter()
{
    join();
}

void MultiThreadVideoWriter::join()
{
    std::cout << "Multi thread video writer : Terminating..." << std::endl;
    is_writing = false;
    th1.join();
    std::cout << "Multi thread video writer : Done." << std::endl;
}

int MultiThreadVideoWriter::push(UMatPtr &p)
{
    write_data_.push(p);
    return 0;
}

MultiThreadVideoReader::MultiThreadVideoReader(std::string input_path,size_t queue_size) : read_data_(queue_size),video_capture(input_path)
{
    if (!video_capture.isOpened())
    {
        std::cerr << "Video file: " << input_path << " cannot be opened." << std::endl
                  << std::flush;
        throw;
    }
    is_reading = true;
    th1 = std::thread(&MultiThreadVideoReader::videoReaderProcess, this); // Run thread
}

void MultiThreadVideoReader::videoReaderProcess()
{
    while (1)
    {
        UMatPtr umat_src(new cv::UMat(cv::Size(video_capture.get(cv::CAP_PROP_FRAME_WIDTH), video_capture.get(cv::CAP_PROP_FRAME_HEIGHT)),
                                      CV_8UC4,
                                      cv::ACCESS_READ,
                                      cv::USAGE_ALLOCATE_DEVICE_MEMORY)); //mat_src.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
        video_capture >> *umat_src;
        if (umat_src->empty())
        {
            is_reading = false;
            return;
        }
        cv::cvtColor(*umat_src, *umat_src, cv::COLOR_BGR2BGRA);
        read_data_.push(umat_src);

        if (!is_reading)
        {
            return;
        }
    }
}

void MultiThreadVideoReader::join()
{
    std::cout << "Multi thread video reader : Terminating..." << std::endl;
    is_reading = false;
    read_data_.clear();
    th1.join();
    std::cout << "Multi thread video reader : Done." << std::endl;
}

int MultiThreadVideoReader::get(UMatPtr &p)
{
    while (read_data_.empty())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (!is_reading)
        {
            p = nullptr;
            return 1;
        }
    }
    int retval = read_data_.get(p);
    read_data_.pop();
    return retval;
}

MultiThreadVideoReader::~MultiThreadVideoReader()
{
    join();
}

MultiThreadRotationMatrixGenerator::MultiThreadRotationMatrixGenerator(
    VideoPtr video_parameter,
    // ResamplerParameterPtr resampler_parameter,
    FilterPtr filter,
    AngularVelocityPtr measured_angular_velocity,
    Eigen::VectorXd filter_strength,
    std::vector<std::pair<int32_t,double>> sync_table,
    size_t queue_size) : video_parameter(video_parameter),
                                    //    resampler_parameter(resampler_parameter),
                                       filter(filter),
                                       measured_angular_velocity(measured_angular_velocity),
                                       filter_strength(filter_strength),
                                       sync_table(sync_table),
                                     rotation_matrix_(queue_size)
{
    is_reading = true;
    th1 = std::thread(&MultiThreadRotationMatrixGenerator::process, this); // Run thread
}

void MultiThreadRotationMatrixGenerator::process()
{
    for (int frame = 0; frame <= video_parameter->video_frames; ++frame)
    {
        MatrixPtr R(new std::vector<float>(video_parameter->camera_info->height_ * 9));
        // Calculate Rotation matrix for every line
        for (int row = 0, e = video_parameter->camera_info->height_; row < e; ++row)
        {
            double frame_in_row = frame + (video_parameter->camera_info->line_delay_ * (row - video_parameter->camera_info->height_ * 0.5))
            * video_parameter->getFrequency();
            Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(&(*R)[row * 9], 3, 3) 
            = measured_angular_velocity->getCorrectionQuaternionFromFrame(frame_in_row,filter->getFilterCoefficient(filter_strength(frame)),sync_table).matrix().cast<float>();
            
            // double time_in_row = video_parameter->getInterval() * frame + resampler_parameter->start + video_parameter->camera_info->line_delay_ * (row - video_parameter->camera_info->height_ * 0.5);
            // printf("frame_in_row:%f time_in_row%f\r\n",measured_angular_velocity->convertEstimatedToMeasuredAngularVelocityFrame(frame_in_row,sync_table)*measured_angular_velocity->getInterval(),time_in_row);

        }

        rotation_matrix_.push(R);

        if (!is_reading)
        {
            break;
        }
        // std::cout << "\r\n frame: " << frame << std::endl << std::flush;
    }
}

int MultiThreadRotationMatrixGenerator::get(MatrixPtr &p)
{
    while (rotation_matrix_.empty())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (!is_reading)
        {
            p = nullptr;
            return 1;
        }
        std::cout << "Empty:" << __FILE__ << " : line " << __LINE__ << std::endl;
    }
    int retval = rotation_matrix_.get(p);
    rotation_matrix_.pop();
    return retval;
}

void MultiThreadRotationMatrixGenerator::join()
{
    std::cout << "Multi thread rotation matrix generator : Terminating..." << std::endl;
    is_reading = false;
    rotation_matrix_.clear();
    th1.join();
    std::cout << "Multi thread rotation matrix generator : Done." << std::endl;
}

MultiThreadRotationMatrixGenerator::~MultiThreadRotationMatrixGenerator()
{
    join();
}