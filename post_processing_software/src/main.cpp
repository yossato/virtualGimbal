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

#include <stdio.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <memory>
#include "virtual_gimbal_manager.h"
#include "json_tools.hpp"
#include "rotation_param.h"
#include "distortion.h"
#include <sys/types.h>
#include <sys/stat.h>

#include "data_collection.h"
#include "point_pairs.hpp"


using namespace std;

int main(int argc, char **argv)
{
    // Check arguments
    char *videoPass = NULL;
    char *cameraName = NULL;
    char *lensName = NULL;
    char *jsonPass = NULL;
    bool output = false;
    bool show_image = true;
    bool inpainting = false;
    const char *kernel_name = "cl/stabilizer_kernel.cl";
    const char *kernel_function = "stabilizer_function";
    double zoom = 1.0;
    int32_t fileter_length = 199;
    int opt;
    int queue_size = 10;
    size_t buffer_size = 21;
    bool analyze = false;
    char *experimental_param_json_path = NULL;
    double sync_period = 30.0;
    int32_t sync_frame_length = 999;
    double subframe_resolution = 0.1;
    double ra4_thresh = 0.6;
    double curve_fitting_valid_value_thresh = 0.3;
    while ((opt = getopt(argc, argv, "j:i:c:l:w:q:z:k:f:o::n::b:a::p:s:x:r:t:m:")) != -1)
    {
        switch (opt)
        {
        case 'j': //input json file from virtual gimbal
            jsonPass = optarg;
            break;
        case 'i': //input video file pass
            videoPass = optarg;
            printf("videoPass %s\r\n",videoPass);
            break;
        case 'c':
            cameraName = optarg;
            break;
        case 'l':
            lensName = optarg;
            break;
        case 'z':       //zoom ratio, dafault 1.0
            zoom = std::stof(optarg);
            break;
        case 'w':
            fileter_length = std::stoi(optarg);
            break;
        case 'q':
            queue_size = std::stoi(optarg);
            break;
        case 'k':
            kernel_name = optarg;
            break;
        case 'f':
            kernel_function = optarg;
            break;
        case 'o':
            output = true;
            break;
        case 'n':
            show_image = false;
            break;
        case 'a':
            analyze = true;
            break;
        case 'p':   // sync period
            sync_period = std::stof(optarg);
            break;
        case 's':   // sync frame length
            sync_frame_length = std::stoi(optarg);
            break;
        case 'x':
            experimental_param_json_path = optarg;
            break;
        case 'r':
            subframe_resolution = std::stof(optarg);
            break;
        case 't':
            ra4_thresh = std::stof(optarg);
            break;
        case 'm':
            curve_fitting_valid_value_thresh = std::stof(optarg);
            break;
        // case 'p':
        //     inpainting = true;
        //     break;
        // case 'b':
        //     buffer_size = std::stoi(optarg);
        //     break;
        default:
            //            printf(     "virtualGimbal\r\n"
            //                        "Hyper fast video stabilizer\r\n\r\n"
            //                        "usage: virtualGimbal [-i video] [-f angularVelocity] [[output option] -o] [options]\r\n"
            //                        );
            return 1;
        }
    }

    struct stat st;
    if (stat(kernel_name, &st))
    {
        std::cerr << "ERROR: Kernel file not found. " << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
        std::exit(EXIT_FAILURE);
    }

    if (stat(videoPass, &st))
    {
        std::cerr << "ERROR: " << videoPass << " is not found. "  << std::endl << std::flush;
        std::exit(EXIT_FAILURE) ;
    }


    VirtualGimbalManager manager(queue_size);
    manager.kernel_function = kernel_function;
    manager.kernel_name = kernel_name;

    // TODO:Check kernel availability here. Build once.

    shared_ptr<CameraInformation> camera_info;
    try
    {
        camera_info = shared_ptr<CameraInformation>( new CameraInformationJsonParser(cameraName, lensName, VirtualGimbalManager::getVideoSize(videoPass).c_str()));        
    }
    catch(std::string e)
    {
        std::cerr << "Error: " << e <<  std::endl;
        std::exit(EXIT_FAILURE);
    }
    calcInverseDistortCoeff(*camera_info);
    manager.setMeasuredAngularVelocity(jsonPass, camera_info);
    manager.setVideoParam(videoPass, camera_info);

    if(output){
        try
        {
            manager.enableWriter(videoPass);
        }
        catch(const char& e)
        {
            std::cerr << "Error: " << e << std::endl;
            std::exit(EXIT_FAILURE);
        }
        
    }

    
    PointPairs feature_points_pairs; // Store feature points at frame [n] and frame [n-1]
    Eigen::MatrixXd estimated_angular_velocity,confidence;

    // Read feature points of video from json file.
    if(pointPairsJsonExists(videoPass))
    {
        if(readPointPairsFromJson(videoNameToPointPairsJsonName(videoPass),feature_points_pairs))
        {
            std::cerr << std::string("Failed to read ") + videoNameToPointPairsJsonName(videoPass) << std::endl;
        }
        manager.estimateAngularVelocity(feature_points_pairs,estimated_angular_velocity,confidence);
    }
    else if(jsonExists(videoPass))// Obsolute
    {
        manager.getAngularVelocityFromJson(estimated_angular_velocity,confidence);
    }
    // If point pairs Json file doesn't exists, create it.
    else 
    {
        feature_points_pairs = manager.getFeaturePointsPairs();
        
        manager.estimateAngularVelocity(feature_points_pairs,estimated_angular_velocity,confidence);

        if(analyze)
        {
            writePointPairesToJson(videoNameToPointPairsJsonName(videoPass),feature_points_pairs);
        }


    }

    if(analyze)
    {
        LoggingDouble d;
        for(int r=0;r<estimated_angular_velocity.rows();++r)
        {
            d["Frame"].push_back((double)r);
            d["rx"].push_back(estimated_angular_velocity(r,0));
            d["ry"].push_back(estimated_angular_velocity(r,1));
            d["rz"].push_back(estimated_angular_velocity(r,2));
        }
        std::string time_stamp = DataCollection::getSystemTimeStamp();
        DataCollection collection(time_stamp + "_estimated_angular_velocity.csv");
        collection.setDuplicateFilePath("latest_estimated_angular_velocity.csv");
        collection.set(d);
    }
    
    manager.setEstimatedAngularVelocity(estimated_angular_velocity, confidence);

    auto fir_filter = std::make_shared<NormalDistributionFilter>();
    manager.setFilter(fir_filter);
    manager.setMaximumGradient(0.5);

    SyncTable table;
    
    // Uncomment before flight
    // if(syncTableJsonExists(videoPass))
    // {
    //     readSyncTableFromJson(videoNameToSyncTableJsonName(videoPass),table);
    // }
    {
        table = manager.getSyncTableRobust(zoom,fir_filter,fileter_length,feature_points_pairs,sync_period,sync_frame_length/240.0, ra4_thresh, subframe_resolution, curve_fitting_valid_value_thresh);
        if(table.size())
        {
            std::cout << "Robust table size:" << table.size() << std::endl;
            printf("Robust table:\r\n");
            LoggingDouble d;
            for(size_t i=0;i<table.size()-1;++i)
            {
                printf("(%d,%f)-(%d,%f), a:%f b=%f\r\n",table[i].first,table[i].second,table[i+1].first,table[i+1].second,
                (table[i+1].second-table[i].second)/(table[i+1].first-table[i].first),
                (table[i].second*table[i+1].first-table[i].first*table[i+1].second)/(table[i+1].first-table[i].first)
                );
                double a = (table[i+1].second-table[i].second)/(table[i+1].first-table[i].first);
                double b = (table[i].second*table[i+1].first-table[i].first*table[i+1].second)/(table[i+1].first-table[i].first);
                d["Estimated angular velocity frame"].push_back(table[i].first);
                d["Measured angular velocity frame"].push_back(table[i].second);
                d["a-skew"].push_back(a);
                d["b-offset"].push_back(b);
            }
            d["Estimated angular velocity frame"].push_back(table.back().first);
            d["Measured angular velocity frame"].push_back(table.back().second);
            d["a-skew"].push_back(0);
            d["b-offset"].push_back(0);

            std::string time_stamp = DataCollection::getSystemTimeStamp();
            DataCollection collection(time_stamp + "_incremental_sync_table.csv");
            collection.setDuplicateFilePath("latest_incremental_sync_table.csv");
            collection.set(d);  
        }
    }

    if(table.empty()){
        table = manager.getSyncTableIncrementally(zoom,fir_filter,fileter_length,feature_points_pairs,sync_period,sync_frame_length/240.0, ra4_thresh);
        if(table.size())
        {
            std::cout << "Incremental table size:" << table.size() << std::endl;
            printf("Estimated new table:\r\n");
            LoggingDouble d;
            for(size_t i=0;i<table.size()-1;++i)
            {
                printf("(%d,%f)-(%d,%f), a:%f b=%f\r\n",table[i].first,table[i].second,table[i+1].first,table[i+1].second,
                (table[i+1].second-table[i].second)/(table[i+1].first-table[i].first),
                (table[i].second*table[i+1].first-table[i].first*table[i+1].second)/(table[i+1].first-table[i].first)
                );
                double a = (table[i+1].second-table[i].second)/(table[i+1].first-table[i].first);
                double b = (table[i].second*table[i+1].first-table[i].first*table[i+1].second)/(table[i+1].first-table[i].first);
                d["Estimated angular velocity frame"].push_back(table[i].first);
                d["Measured angular velocity frame"].push_back(table[i].second);
                d["a-skew"].push_back(a);
                d["b-offset"].push_back(b);
            }
            d["Estimated angular velocity frame"].push_back(table.back().first);
            d["Measured angular velocity frame"].push_back(table.back().second);

            std::string time_stamp = DataCollection::getSystemTimeStamp();
            DataCollection collection(time_stamp + "_incremental_sync_table.csv");
            collection.setDuplicateFilePath("latest_incremental_sync_table.csv");
            collection.set(d);  
        }
    }

    if(table.empty()){ // Experimental
        
        /*SyncTable */table = manager.getSyncTable(zoom,fir_filter,fileter_length,feature_points_pairs,sync_period,sync_frame_length/240.0);
        
        printf("Estimated new table:\r\n");
        LoggingDouble d;
        for(size_t i=0;i<table.size()-1;++i)
        {
            printf("(%d,%f)-(%d,%f), a:%f b=%f\r\n",table[i].first,table[i].second,table[i+1].first,table[i+1].second,
            (table[i+1].second-table[i].second)/(table[i+1].first-table[i].first),
            (table[i].second*table[i+1].first-table[i].first*table[i+1].second)/(table[i+1].first-table[i].first)
            );
            double a = (table[i+1].second-table[i].second)/(table[i+1].first-table[i].first);
            double b = (table[i].second*table[i+1].first-table[i].first*table[i+1].second)/(table[i+1].first-table[i].first);
            d["Estimated angular velocity frame"].push_back(table[i].first);
            d["Measured angular velocity frame"].push_back(table[i].second);
            d["a-skew"].push_back(a);
            d["b-offset"].push_back(b);
        }
        d["Estimated angular velocity frame"].push_back(table.back().first);
        d["Measured angular velocity frame"].push_back(table.back().second);

        std::string time_stamp = DataCollection::getSystemTimeStamp();
        DataCollection collection(time_stamp + "_new_sync_table.csv");
        collection.setDuplicateFilePath("latest_new_sync_table.csv");
        collection.set(d);  
    }
    
    if(table.empty()){
        table = manager.getSyncTable(sync_period,sync_frame_length);
        if(2 >table.size()){
            printf("Warning: Input video too short to apply poly line syncronize method, an alternative mothod is used.\r\n");
            table = manager.getSyncTableOfShortVideo();
        }
        printf("Table:\r\n");
        for(size_t i=0;i<table.size()-1;++i)
        {
            printf("(%d,%f)-(%d,%f), a:%f b=%f\r\n",table[i].first,table[i].second,table[i+1].first,table[i+1].second,
            (table[i+1].second-table[i].second)/(table[i+1].first-table[i].first),
            (table[i].second*table[i+1].first-table[i].first*table[i+1].second)/(table[i+1].first-table[i].first)
            );
        }
        if(0){
            auto copied_table = table;
            for(auto &el:copied_table)
            {
                el.second += (double)el.first * -0.0003278362805483809 + 1.1475274725274724;
            }
            writeSyncTableToJson(videoNameToSyncTableJsonName(videoPass),copied_table);

        }
        writeSyncTableToJson(videoNameToSyncTableJsonName(videoPass),table);
    }




    if(analyze)
    {
        LoggingDouble d;
        for(size_t i=0;i<table.size()-1;++i)
        {
            double a = (table[i+1].second-table[i].second)/(table[i+1].first-table[i].first);
            double b = (table[i].second*table[i+1].first-table[i].first*table[i+1].second)/(table[i+1].first-table[i].first);
            d["Estimated angular velocity frame"].push_back(table[i].first);
            d["Measured angular velocity frame"].push_back(table[i].second);
            d["a-skew"].push_back(a);
            d["b-offset"].push_back(b);
        }
        std::string time_stamp = DataCollection::getSystemTimeStamp();
        DataCollection collection(time_stamp + "_sync_table.csv");
        collection.setDuplicateFilePath("latest_sync_table.csv");
        collection.set(d);
    }


    Eigen::VectorXd filter_coefficients = manager.getFilterCoefficients(zoom,fir_filter,table,fileter_length,0); // Zero is the weakest value since apply no filter, output is equal to input.

    if(analyze)
    {
        LoggingDouble d;
        for(int r=0;r<filter_coefficients.rows();++r)
        {
            d["Frame"].push_back((double)r);
            d["Filter coefficients"].push_back(filter_coefficients(r,0));
        }
        std::string time_stamp = DataCollection::getSystemTimeStamp();
        DataCollection collection(time_stamp + "_filter_coefficients.csv");
        collection.setDuplicateFilePath("latest_filter_coefficients.csv");
        collection.set(d);
        
       

    }

    if(analyze)
    {
        // manager.spinAnalyse(zoom,fir_filter,filter_coefficients,table,feature_points_pairs);
        manager.spinAnalyse(1.f,fir_filter,filter_coefficients,table,feature_points_pairs,experimental_param_json_path);
        return 0;
    }

    if(inpainting)
    {
        manager.spinInpainting(zoom,table,fir_filter,buffer_size);
    }
    else
    {
        manager.spin(zoom,fir_filter,filter_coefficients,table, show_image);    
    }
    

    return 0;
}