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
#ifndef __VIRTUALGIMBAL_DATA_COLLECTION_H__
#define __VIRTUALGIMBAL_DATA_COLLECTION_H__

#include <string>
// #include <unistd.h>
#include <vector>
// #include <sys/stat.h>
#include <map>
#include <ctime>
#include <chrono>
#include <assert.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <unistd.h>

using LoggingDouble = std::map<std::string, std::vector<double>>;
using LoggingTime = std::vector<timespec>;

class DataCollection
{
public:
    DataCollection();       // Dafault constructor, file path is automatically generated.
    DataCollection(std::string file_path);  // Constructor with file path.
    ~DataCollection();      // Destructor
    void set(LoggingTime &time);
    void set(LoggingDouble &value);
    void setDuplicateFilePath(std::string duplicate_file_path);
    
    void print();
    
    std::string getValueTimeStamp();
    static std::string getSystemTimeStamp();
    

private:
    LoggingDouble values_;
    LoggingTime time_;
    std::string file_path_;
    std::string duplicate_file_path_;
    std::string valuesToString();
    void write(std::string file_path);
};

#endif //__VIRTUALGIMBAL_DATA_COLLECTION_H__