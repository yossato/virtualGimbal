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
            auto el = Quaternion2Vector(angle_quaternion[i]);//require Quaternion2Matrix<3,1>()
            raw_angle(0,i) = el[0];
            raw_angle(1,i) = el[1];
            raw_angle(2,i) = el[2];
        }
        is_filterd = false;
    }

    vector<double> getRow(int r);

    const Eigen::MatrixXd data();

    template <class T> void setFilterCoeff(T coeff){
        filter_coeff.resize(1,coeff.size());
        for(int i=0,e=coeff.size();i<e;++i){
            filter_coeff(0,i) = coeff[i];
        }
    }

    const Eigen::MatrixXd &filterdData();

private:
    Eigen::MatrixXd raw_angle;
    Eigen::MatrixXd filterd_angle;
    Eigen::MatrixXd filter_coeff;
    bool is_filterd;
};

#endif // VSP_H
