/**
* This file is part of VirtualGimbal.
*
* Copyright 2019 Yoshiaki Sato <virtualgimbal at xa2 dot so-net dot ne dot jp>
*
* VirtualGimbal is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.

* VirtualGimbal is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.

* You should have received a copy of the GNU General Public License
* along with VirtualGimbal.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "rotation_param.h"

const double BaseParam::getFrequency(){
    return frequency_;
}

const double BaseParam::getInterval(){
    return 1./frequency_;
}

Eigen::VectorXd BaseParam:: operator()(int32_t index){
    return data.row(index).transpose();
}

Video::Video(double frequency)
{
    frequency_ = frequency;
}

AngularVelocity::AngularVelocity(double frequency){
    frequency_ = frequency;
}

Rotation::~Rotation(){

}

Eigen::Quaterniond Rotation::getDiffQuaternion(double index){
    return Eigen::Quaterniond();
}
