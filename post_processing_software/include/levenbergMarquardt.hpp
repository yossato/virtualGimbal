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
#ifndef LEVENBERGMARQUARDT_HPP
#define LEVENBERGMARQUARDT_HPP

#include <iostream>

#include "Eigen/Dense"
#include "unsupported/Eigen/NonLinearOptimization"
#include "unsupported/Eigen/NumericalDiff"
#include <opencv2/opencv.hpp>
#include "calcShift.hpp"
using namespace Eigen;

// Generic functor
template<typename _Scalar, int NX=Dynamic, int NY=Dynamic>
struct Functor
{
  typedef _Scalar Scalar;
  enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;
  
  Functor() : inputs_(InputsAtCompileTime), values_(ValuesAtCompileTime){}
  Functor(int inputs, int values) : inputs_(inputs), values_(values) {}
  
  int inputs() const { return inputs_; }
  int values() const { return values_; }
  
  const int inputs_;
  const int values_;

  
};

/**
 * @brief OpenCV等の歪補正の係数に関して逆方向に変換する逆係数を計算します。
 * @param [in] undistortedPoints	OpenCVなどで歪補正した、歪補正済み画像上の点の座標の組
 * @param [in] refPoints			歪補正前の歪んだ画像上の点の座標の組
 **/
struct calc_invert_distortion_coeff : Functor<double>
{
	calc_invert_distortion_coeff(int inputs, int values, std::vector<double> &undistortedPointsX, std::vector<double> &undistortedPointsY, std::vector<double> &refPointsX, std::vector<double> &refPointsY, Matrix3d &intrinsicCoeff)
	: Functor(inputs, values), m_undistortedPointsX(undistortedPointsX),m_undistortedPointsY(undistortedPointsY), m_refPointsX(refPointsX), m_refPointsY(refPointsY), m_intrinsicCoeff(intrinsicCoeff)  {}
	
	std::vector<double> m_undistortedPointsX;
	std::vector<double> m_undistortedPointsY;
	std::vector<double> m_refPointsX;
	std::vector<double> m_refPointsY;
	Matrix3d m_intrinsicCoeff;

	int operator()(const VectorXd& K, VectorXd& fvec) const
    {
		double fx = m_intrinsicCoeff(0, 0);
		double fy = m_intrinsicCoeff(1, 1);
		double cx = m_intrinsicCoeff(0, 2);
		double cy = m_intrinsicCoeff(1, 2);
		
		double k1 = K[0];
		double k2 = K[1];
		double p1 = K[2];
		double p2 = K[3];
		
		for(int i=0,e=values_;i<e;i++){
			
			double u = m_undistortedPointsX[i];
			double v = m_undistortedPointsY[i];

			
			//後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
			//~ cv::Mat p = (cv::Mat_<double>(3,1) << (u - cx)/fx, (v - cy)/fy, 1.0);	//1のポリゴン座標に、K^-1を掛けた結果の３x１行列
			//~ cv::Mat XYW = R.inv() * p;
			
			//~ double x1 = XYW.at<double>(0, 0)/XYW.at<double>(2, 0);
			//~ double y1 = XYW.at<double>(1, 0)/XYW.at<double>(2, 0);
			double x1 = (u - cx)/fx;
			double y1 = (v - cy)/fy;
			
			double r = sqrt(x1*x1+y1*y1);
			
			double x2 = x1*(1.0+k1*r*r+k2*r*r*r*r)+2.0*p1*x1*y1+p2*(r*r+2.0*x1*x1);
			double y2 = y1*(1.0+k1*r*r+k2*r*r*r*r)+p1*(r*r+2.0*y1*y1)+2.0*p2*x1*y1;
			double mapx = x2*fx+cx;
			double mapy = y2*fy+cy;
			
			fvec[i] 	= (mapx - m_refPointsX[i])*(mapx - m_refPointsX[i]) + (mapy - m_refPointsY[i])*(mapy - m_refPointsY[i]);
			//~ fvec[i] 	= mapx - m_refPointsX[i];
			//~ fvec[i+1] 	= mapy - m_refPointsY[i];
		}

        return 0;
    }
};
#endif
