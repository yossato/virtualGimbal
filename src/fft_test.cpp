#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include "matplotlib-cpp/matplotlibcpp.h"
#include <math.h>
namespace plt = matplotlibcpp;

int main(int argc, char** argv){
    int L = 300;
    int N = L*2;
    double t=1;
    Eigen::FFT<double> fft;
    std::vector<double> time_vec(N),time(N);
    std::vector<std::complex<double>> freq_vec;
    for(int i=0,e=time_vec.size();i<e;++i){
        time[i] = i;
        time_vec[i] = 1.0;
        time_vec[i] += 0.5*cos(2.0*M_PI*t*(double)i/(double)N);
        time_vec[i] += 0.2*cos(2.0*M_PI*30*(double)i/(double)N);
        time_vec[i] += 0.1*cos(2.0*M_PI*7*(double)i/(double)N);
    }


    std::vector<double> vec_real(N),vec_imag(N);

    fft.fwd(freq_vec,time_vec);

    fft.inv(time_vec,freq_vec);


    for(int i=0,e=time_vec.size();i<e;++i){
        vec_real[i] = freq_vec[i].real();
        vec_imag[i] = freq_vec[i].imag();
    }
    plt::plot(time,time_vec,".-r");
    plt::show();
    plt::plot(time,vec_real,".-g");
    plt::plot(time,vec_imag,".-b");
    plt::show();

    return 0;
}
