/*************************************************************************
*  Software License Agreement (BSD 3-Clause License)
*  
*  Copyright (c) 2022, Yoshiaki Sato
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
#include "data_collection.h"


DataCollection::DataCollection()
{
}

DataCollection::DataCollection(std::string file_path) : file_path_(file_path)
{
        
}

DataCollection::~DataCollection()
{
    // If file path is not defined, it set default value with time stamp.
    if(file_path_.empty())
    {
        if(!time_.empty())
        {
            file_path_ = getValueTimeStamp() + "/log.csv";
        }
        else
        {
            file_path_ = getSystemTimeStamp() + "/log.csv";
        }
    }

    write(file_path_);

    if(!duplicate_file_path_.empty())
    {
        write(duplicate_file_path_);
    }
}

void DataCollection::set(LoggingTime &time)
{
    // if(!time_.empty()) // You can call set function only once.
    // {
    //     std::cerr << "Error: You called set function more than once." << std::endl;
    // }
    time_ = time;
}

void DataCollection::set(LoggingDouble &value)
{
    values_ = value;
}

void DataCollection::setDuplicateFilePath(std::string duplicate_file_path)
{
    duplicate_file_path_ = duplicate_file_path;
}

void DataCollection::write(std::string file_path)
{
    std::ofstream ofs(file_path);
    ofs << valuesToString();
    ofs.close();
}

void DataCollection::print()
{
    std::cout << valuesToString() << std::endl;
}

std::string DataCollection::getValueTimeStamp()
{
    char buf[32];
    struct tm t;
    if (!time_.empty()) // time_があればそのデータの末尾を使う
    {
        localtime_r(&(time_.begin()->tv_sec), &t);
    }
    else
    {
        return std::string("Failed_to_get_value_time_stamp.");
    }

    strftime(buf, 32, "%Y-%m-%d_%H-%M-%S", &t);
    return std::string(buf);
}

std::string DataCollection::getSystemTimeStamp()
{
    char buf[32];
    struct tm t;
    
    timespec ts;
    if (timespec_get(&ts, TIME_UTC) == 0)
    {
        return std::string("Failed_to_get_current_time.");
    }
    localtime_r(&(ts.tv_sec), &t);
    

    strftime(buf, 32, "%Y-%m-%d_%H-%M-%S", &t);
    return std::string(buf);
}

std::string DataCollection::valuesToString()
{
    std::string retstr;
    if (values_.empty())
    {
        std::cerr << "Error: There is no data for converting to CSV." << std::endl;
    }

    if (!time_.empty())
    {
        assert(time_.size() == values_.begin()->second.size()); // Timestamp and first value are same size if time exists.
    }
    for (const auto &el : values_)
    {
        assert(values_.begin()->second.size() == el.second.size()); // All values are same size.
    }

    // First row with item names.
    if (!time_.empty())
    {
        retstr += "time,";
    }
    for (const auto &el : values_)
    {
        retstr += el.first + ","; // Each item's name
    }
    retstr.pop_back();  // Remove an extra comma.
    retstr += "\r\n";

    // Values
    for (size_t i = 0; i < values_.begin()->second.size(); ++i)
    {
        // Timestamp
        if (!time_.empty())
        {
            // second
            retstr += std::to_string(time_[i].tv_sec) + std::string(".");
            // nano second
            std::ostringstream sout;
            sout << std::setfill('0') << std::setw(9) << time_[i].tv_nsec;
            retstr += sout.str() + ",";
        }

        // Values
        for (const auto &el : values_)
        {
            retstr += std::to_string(el.second[i]) + ",";
        }
        retstr.pop_back();  // Remove an extra comma.

        retstr += "\r\n";
    }

    return retstr;
}
