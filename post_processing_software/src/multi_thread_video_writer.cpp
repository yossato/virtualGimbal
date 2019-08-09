#include "multi_thread_video_writer.h"

// UMatWithMutex::UMatWithMutex(UMatPtr &p){
//     ptr = std::move(p);
// }



int MultiThreadVideoData::push(UMatPtr &p){
    while(1){
        size_t data_size;
        {
            std::lock_guard<std::mutex> lock(mutex);
            data_size = data.size();
        }
        if(data_size < max_size_){
            break;
        }else{
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // std::cout << "Waiting, data.size:" << data.size() << std::endl;
        }
    }

    std::lock_guard<std::mutex> lock(mutex);
    data.emplace(std::move(p));
    return 0;
}

int MultiThreadVideoData::get(UMatPtr &p){
    std::lock_guard<std::mutex> lock(mutex);
    p = std::move(data.front());
    return 0;
}

int MultiThreadVideoData::pop(){
    std::lock_guard<std::mutex> lock(mutex);
    if(data.empty())
    {
        return 1;
    }
    data.pop();
    // std::cout << "data size:" << data.size() << std::endl;
    return 0;
}

bool MultiThreadVideoData::empty(){
    std::lock_guard<std::mutex> lock(mutex);
    return !data.size();
}

void MultiThreadVideoData::clear()
{
    std::lock_guard<std::mutex> lock(mutex);
    while(!data.empty()){
        data.pop();
    }
}

// int MultiThreadVideoData::size(){
//     std::lock_guard<std::mutex> lock(mutex);
//     return data.size();
// }

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
    th1 = std::thread(&MultiThreadVideoWriter::videoWriterProcess, this); // Run thread
}

void MultiThreadVideoWriter::videoWriterProcess()
{
    while (1)
    { 
        if(!write_data_.empty()){
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

int MultiThreadVideoWriter::push(UMatPtr &p){
    write_data_.push(p);
    return 0;
}

MultiThreadVideoReader::MultiThreadVideoReader(std::string input_path) : video_capture(input_path){
    if(!video_capture.isOpened())
    {
        std::cerr << "Video file: " << input_path << " cannot be opened." << std::endl << std::flush;
        throw;
    }
    is_reading = true;
    th1 = std::thread(&MultiThreadVideoReader::videoReaderProcess, this); // Run thread
}

void MultiThreadVideoReader::videoReaderProcess()
{
    while (1)
    {
        UMatPtr umat_src(new cv::UMat(cv::Size(video_capture.get(cv::CAP_PROP_FRAME_WIDTH),video_capture.get(cv::CAP_PROP_FRAME_HEIGHT)), CV_8UC4, cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY));//mat_src.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
        video_capture >> *umat_src;
        if(umat_src->empty()){
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

int MultiThreadVideoReader::get(UMatPtr &p){
    while(read_data_.empty())
    {  
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if(!is_reading)
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