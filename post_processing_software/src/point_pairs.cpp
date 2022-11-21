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

#include "point_pairs.hpp"


std::string sample_point_pairs = "{\"point_pairs\":[[[[1.0,1.1],[2.0,2.1]]],[[[3.0,3.1],[4.0,4.1]]]]}";

using namespace rapidjson;
using namespace Eigen;

bool pointPairsJsonExists(std::string video_file_name)
{
    std::string json_file_name = videoNameToPointPairsJsonName(video_file_name);
    struct stat st;
    return !stat(json_file_name.c_str(), &st);
}

std::string videoNameToPointPairsJsonName(std::string video_name)
{
    std::string json_file_name = video_name;
    std::string::size_type pos;
    if ((pos = json_file_name.find_last_of(".")) != std::string::npos)
    {
        //拡張子の長さをチェック
        if ((json_file_name.length() - json_file_name.substr(0, pos).length()) > 4)
        {
            json_file_name += std::string("_pp.json");
        }
        else
        {
            //拡張子の判定が3文字以下なら拡張子を削除して.jsonをつける
            json_file_name = json_file_name.substr(0, pos) + std::string("_pp.json");
        }
    }
    return json_file_name;
}

int readPointPairsFromJson(std::string json_path, PointPairs &point_pairs)
{

    FILE *fp = fopen(json_path.c_str(), "rb"); // non-Windows use "r"
    char readBuffer[262140];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document e;
    e.ParseStream(is);
    fclose(fp);

    const Value &point_pairs_array = e["point_pairs"];
    for (auto &pair:point_pairs_array.GetArray())
    {
        PointPair p;
        // Points on previous image
        for(auto &prev:pair[0].GetArray())
        {
            p.first.push_back(cv::Point2f(prev[0].GetFloat(),prev[1].GetFloat()));
        }
        for(auto &curr:pair[1].GetArray())
        {  
            p.second.push_back(cv::Point2f(curr[0].GetFloat(),curr[1].GetFloat()));
        }

        point_pairs.push_back(p);
    }

    return 0;
}

int writePointPairesToJson(std::string json_path, const PointPairs &point_pairs)
{
    Document d(kObjectType);
    Document::AllocatorType &d_allocator = d.GetAllocator();

    Document pairs(kArrayType);
    // Document::AllocatorType &pairs_allocator = pairs.GetAllocator();
    for(const auto &p:point_pairs)
    {

        Document pair(kArrayType);
        // Document::AllocatorType &pair_allocator =  pair.GetAllocator();

        // Points on previous image
        Document prev(kArrayType);
        // Document::AllocatorType &prev_allocator = prev.GetAllocator();
        for(const auto &p_prev:p.first)
        {
            Document point(kArrayType);
            // Document::AllocatorType &point_allocator = point.GetAllocator();
            point.PushBack(p_prev.x,d_allocator);
            point.PushBack(p_prev.y,d_allocator);
            prev.PushBack(point,d_allocator);
        }

        // Point on current image
        Document curr(kArrayType);
        // Document::AllocatorType &curr_allocator = curr.GetAllocator();
        for(const auto &p_curr:p.second)
        {
            Document point(kArrayType);
            // Document::AllocatorType &point_allocator = point.GetAllocator();
            point.PushBack(p_curr.x,d_allocator);
            point.PushBack(p_curr.y,d_allocator);
            curr.PushBack(point,d_allocator);
        }

        pair.PushBack(prev, d_allocator);
        pair.PushBack(curr, d_allocator);

        pairs.PushBack(pair, d_allocator);

    }

    d.AddMember("point_pairs", pairs, d_allocator);



    FILE *fp = fopen(json_path.c_str(), "wb"); // non-Windows use "w"

    char writeBuffer[262140];
    FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
    Writer<FileWriteStream> writer(os);
    d.Accept(writer);
    fclose(fp);
    return 0;


}

int readSyncTableFromJson(std::string json_path, SyncTable &sync_table)
{
    FILE *fp = fopen(json_path.c_str(), "rb"); // non-Windows use "r"
    char readBuffer[262140];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document e;
    e.ParseStream(is);
    fclose(fp);

    if(!e.HasMember("sync_table"))
    {
        return 1;
    }

    const Value &table = e["sync_table"];
    for (auto &pair:table.GetArray())
    {
        std::pair<int32_t, double> p(pair[0].GetInt(),pair[1].GetFloat());
        sync_table.push_back(p);
    }
    return 0;
}

int writeSyncTableToJson(std::string json_path, const SyncTable &sync_table)
{
    // Document d(kObjectType);

    // If record json file exist, Open it.
    Document d;
    struct stat st;
    if (!stat(json_path.c_str(), &st))
    {
        FILE *fp = fopen(json_path.c_str(), "rb"); // non-Windows use "r"
        std::vector<char> readBuffer((intmax_t)st.st_size + 10);
        FileReadStream is(fp, readBuffer.data(), readBuffer.size());

        d.ParseStream(is);
        fclose(fp);
    }
    else
    {
        d = Document(kObjectType);
    }


    Document::AllocatorType &d_allocator = d.GetAllocator();

    Document table(kArrayType);
    // Document::AllocatorType &pairs_allocator = pairs.GetAllocator();
    for(const auto &st:sync_table)
    {
        Document pair(kArrayType);
        // Document::AllocatorType &point_allocator = point.GetAllocator();
        pair.PushBack(st.first,d_allocator);
        pair.PushBack(st.second,d_allocator);
        table.PushBack(pair,d_allocator);
    }

    d.AddMember("sync_table", table, d_allocator);

    FILE *fp = fopen(json_path.c_str(), "wb"); // non-Windows use "w"

    char writeBuffer[262140];
    FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
    Writer<FileWriteStream> writer(os);
    d.Accept(writer);
    fclose(fp);
    return 0;
}
