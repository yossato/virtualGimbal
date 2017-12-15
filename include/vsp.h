#ifndef VSP_H
#define VSP_H

#include <stdio.h>
#include <boost/math/quaternion.hpp>
#include "settings.h"
#include <Eigen/Dense>

using namespace std;
using namespace boost::math;

class vsp
{
public:
    vsp();

    template <class T> vsp(vector<quaternion<T>> &angle_quaternion){
        raw_angle.resize(3,angle_quaternion.size());

        for(int i=0,e=angle_quaternion.size();i<e;++i){
            auto el = Quaternion2Vector(angle_quaternion[i]);
            raw_angle(0,i) = el[0];
            raw_angle(1,i) = el[1];
            raw_angle(2,i) = el[2];
        }
    }

    vector<double> get_row(int r);


private:
    Eigen::MatrixXd raw_angle;
};

#endif // VSP_H
