#ifndef __CL_MANAGER_H__
#define __CL_MANAGER_H__

#include <iostream>
#include <fstream>
#include <string>
#include <iterator>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
void initializeCL(cv::ocl::Context &context);
void getKernel(const char *kernel_code_file_name, const char *kernel_function, cv::ocl::Kernel &kernel, cv::ocl::Context &context, std::string build_opt);


#endif //__CL_MANAGER_H_