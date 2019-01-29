#ifndef VIDEO_ANALYZER_HPP
#define VIDEO_ANALYZER_HPP

#include <stdio.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>
#include <Eigen/Dense>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filewritestream.h"
#include "rapidjson/filereadstream.h"
#include "memory"
#include <sys/types.h>
#include <sys/stat.h>
void writeSynchronizedQuaternion(const Eigen::MatrixXd &raw_quaternion, const std::string video_name);
int readSynchronizedQuaternion( Eigen::MatrixXd &raw_quaternion, const std::string video_name);
bool jsonExists(std::string video_file_name);
int writeOpticalFrowToJson(Eigen::MatrixXd &optical_flow,std::string video_file_name);
int readOpticalFlowFromJson(Eigen::MatrixXd &optical_flow,std::string video_file_name);
std::string videoNameToJsonName(std::string video_name);
#endif // VIDEO_ANALYZER_HPP
