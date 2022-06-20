/*************************************************************************
*  Software License Agreement (BSD 3-Clause License)
*  
*  Copyright (c) 2019, Yoshiaki Sato
*  All rights reserved.
*  
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*  
*  1. Redistributions of source code must retain the above copyright notice, this
*     list of conditions and the following disclaimer.
*  
*  2. Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*  
*  3. Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived from
*     this software without specific prior written permission.
*  
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*************************************************************************/
#ifndef __ROTATION_MATH_H__
#define __ROTATION_MATH_H__

#include "Eigen/Dense"
#define EPS_Q 1E-4
    
    template <typename T_num>
    Eigen::Vector3d Quaternion2Vector(Eigen::Quaternion<T_num> q, Eigen::Vector3d prev){
        double denom = sqrt(1-q.w()*q.w());
        if(denom==0.0){//まったく回転しない時は０割になるので、場合分けする
            return Eigen::Vector3d(0,0,0);//return zero vector
        }
        double theta_2 = atan2(denom,q.w());
        double prev_theta_2 = prev.norm()/2;
        double diff = theta_2 - prev_theta_2;
        theta_2 -= 2.0*M_PI*(double)(static_cast<int>(diff/(2.0*M_PI)));//マイナスの符号に注意
        //~ printf("Theta_2:%4.3f sc:%d\n",theta_2,static_cast<int>(diff/(2.0*M_PI)));
        if(static_cast<int>(diff/(2.0*M_PI))!=0){
            printf("\n###########Unwrapping %d\n",static_cast<int>(diff/(2.0*M_PI)));
        }

        return  Eigen::Vector3d(q.x(),q.y(),q.z())*2.0*theta_2/denom;
    }
    
    /**
     * @param 回転を表すクォータニオンをシングルローテーションをあらわすベクトルへ変換
     **/
    template <typename T_num>
    Eigen::Vector3d Quaternion2Vector(Eigen::Quaternion<T_num> q)
    {
        double denom = sqrt(1 - q.w() * q.w()); // abs(sin(theta/2))
        if (fabs(denom) < EPS_Q)
        {
            return Eigen::Vector3d(q.x(), q.y(), q.z()) * 2.0; // If an angle is smaller than one, sin(theta/2) can be approximated to theta/2. We can remove denominator to prevent zero devision error.
        }
        return Eigen::Vector3d(q.x(), q.y(), q.z()) * 2.0 * atan2(denom, q.w()) / denom;
    }

    /**
     * @param シングルローテーションを表すベクトルを回転を表すクォータニオンへ変換
     **/
    template <typename T_num>
    Eigen::Quaternion<T_num> Vector2Quaternion(Eigen::Vector3d w)
    {
        double theta = w.norm(); //sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]);//回転角度を計算、normと等しい
        //0割を回避するためにマクローリン展開
        if (theta > EPS_Q)
        {
            Eigen::Vector3d n = w.normalized(); //w * (1.0/theta);//単位ベクトルに変換
            //            double sin_theta_2 = sin(theta*0.5);
            //            return Eigen::Quaternion<T_num>(cos(theta*0.5),n[0]*sin_theta_2,n[1]*sin_theta_2,n[2]*sin_theta_2);
            Eigen::VectorXd n_sin_theta_2 = n * sin(theta * 0.5);
            return Eigen::Quaternion<T_num>(cos(theta * 0.5), n_sin_theta_2[0], n_sin_theta_2[1], n_sin_theta_2[2]);
        }
        else
        {
            return Eigen::Quaternion<T_num>(1.0, 0.5 * w[0], 0.5 * w[1], 0.5 * w[2]).normalized();
        }
    }

#endif //__ROTATION_MATH_H__