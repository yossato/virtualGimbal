#include "multi_thread_video_writer.h"

MultiThreadVideoWriter::MultiThreadVideoWriter(std::string output_pass, Video &video_param)
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
    th1 = std::thread(&MultiThreadVideoWriter::videoWriterProcess, this); //スレッド起動
}

void MultiThreadVideoWriter::videoWriterProcess()
{
    cv::Mat _buf;
    while (1)
    { //繰り返し書き込み
        {
            std::lock_guard<std::mutex> lock(mtx);
            //bufferにデータがあるか確認
            if (images.size() != 0)
            {
                //先頭をコピー
                _buf = images.front().clone();
                //先頭を削除
                images.pop_front();
            }
            else if (!is_writing)
            {
                return;
            }
        }
        //mutexがunlockされたあとにゆっくりvideoWriterに書き込み
        if (!_buf.empty())
        {
            cv::Mat bgr;
            cv::cvtColor(_buf, bgr, cv::COLOR_BGRA2BGR);
            video_writer << bgr;
            _buf = cv::Mat();
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
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

void MultiThreadVideoWriter::addFrame(cv::Mat image)
{
    std::lock_guard<std::mutex> lock(mtx);
    images.push_back(cv::Mat());
    images.back() = image.clone();
}

// void MultiThreadVideoWriter::beginThread(){

// }

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