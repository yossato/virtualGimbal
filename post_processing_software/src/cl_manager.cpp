#include "cl_manager.h"
using namespace std;
void initializeCL(cv::ocl::Context &context)
{
    if (!cv::ocl::haveOpenCL())
    {
        cout << "OpenCL is not avaiable..." << endl
             << flush;
        throw "OpenCL is not avaiable...";
    }
    // cv::ocl::Context context;
    if (!context.create(cv::ocl::Device::TYPE_GPU))
    {
        cout << "Failed creating the context..." << endl;
        throw "Failed creating the context...";
    }

    // In OpenCV 3.0.0 beta, only a single device is detected.
    cout << context.ndevices() << " GPU devices are detected." << endl;
    for (size_t i = 0; i < context.ndevices(); i++)
    {
        cv::ocl::Device device = context.device(i);
        cout << "name                 : " << device.name() << endl;
        cout << "available            : " << device.available() << endl;
        cout << "imageSupport         : " << device.imageSupport() << endl;
        cout << "OpenCL_C_Version     : " << device.OpenCL_C_Version() << endl;
        cout << endl;
    }

    // Select the first device
    cv::ocl::Device(context.device(0));
}

void getKernel(const char *kernel_code_file_name, const char *kernel_function, cv::ocl::Kernel &kernel, cv::ocl::Context &context, std::string &build_opt)
{
    std::ifstream ifs(kernel_code_file_name);
    assert(!ifs.fail() && "kernel code file not found.");
        
    std::string kernelSource((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    cv::ocl::ProgramSource programSource(kernelSource);

    // Compile the kernel code
    cv::String errmsg;
    cv::ocl::Program program = context.getProg(programSource, build_opt, errmsg);
    if (NULL == program.ptr())
    {
        std::cerr << errmsg << std::endl << std::flush;
        throw "getProg failed.";
    }

    kernel = cv::ocl::Kernel(kernel_function, program);
}

void runKernel(cv::UMat &umat_src, cv::UMat &umat_dst,cv::ocl::Kernel &kernel){
    cv::ocl::Image2D image(umat_src);
    cv::ocl::Image2D image_dst(umat_dst,false,true);
    float shift_x = 100.5;
    float shift_y = -50.0;
    // cv::ocl::Kernel kernel("color_shift_invert", program);
    kernel.args(image, image_dst,shift_x,shift_y);

    size_t globalThreads[3] = {(size_t)umat_src.cols, (size_t)umat_src.rows, 1};
    bool success = kernel.run(3, globalThreads, NULL, true);
    if (!success)
    {
        cout << "Failed running the kernel..." << endl << flush;
        throw "Failed running the kernel...";
    }

    // Download the dst data from the device (?)
    // cv::Mat mat_dst = umat_dst.getMat(cv::ACCESS_READ);
}