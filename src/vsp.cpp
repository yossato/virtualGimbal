#include "vsp.h"

vsp::vsp()
{

}

//template <class T> vsp::vsp(vector<quaternion<T>> &angle_quaternion)

vector<double> vsp::get_row(int r){
    vector<double> retval;
    for(int i=0,e=raw_angle.cols();i<e;++i){
        retval.push_back(raw_angle(r,i));
    }
    return retval;
}
