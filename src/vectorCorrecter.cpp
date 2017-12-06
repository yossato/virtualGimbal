#include <stdio.h>
#include <iostream>
#include "matplotlib-cpp/matplotlibcpp.h"
namespace plt = matplotlibcpp;
int main(int argc, char** argv){
    plt::plot({1,2,3,4});
    plt::show();
    std::cout << "Hello world." << std::endl;
}
