#include <stdio.h>
#include <iostream>
#include "matplotlib-cpp/matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace std;

int main(int argc, char** argv){
    plt::plot({1,2,3,4});
    plt::show();

    int n = 5000;
    vector<double> x(n), y(n), z(n);
    for(int i=0; i<n; ++i) {
        x.at(i) = i;
        y.at(i) = sin(2*M_PI*i/360.0);
        z.at(i) = cos(2*M_PI*i/360.0);
    }

    plt::plot(x, y, "xr");
    plt::show();

    std::cout << "Hello world." << std::endl;
}
