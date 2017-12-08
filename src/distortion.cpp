#include <stdio.h>
#include "levenbergMarquardt.hpp"

void calcDistortCoeff(const cv::Mat &matIntrinsic, const cv::Mat &matDistort, const cv::Size &imageSize, cv::Mat &matInvDistort){
    //逆歪パラメータを求める
    std::vector<double> refPointsX;
    std::vector<double> refPointsY;
    Matrix3d intrinsic;
    intrinsic << 	matIntrinsic.at<double>(0,0),matIntrinsic.at<double>(0,1),matIntrinsic.at<double>(0,2),
            matIntrinsic.at<double>(1,0),matIntrinsic.at<double>(1,1),matIntrinsic.at<double>(1,2),
            matIntrinsic.at<double>(2,0),matIntrinsic.at<double>(2,1),matIntrinsic.at<double>(2,2);

    std::cout << intrinsic << std::endl;
    VectorXd distortionCoeff(4);
    distortionCoeff << matDistort.at<double>(0,0),matDistort.at<double>(0,1),matDistort.at<double>(0,2),matDistort.at<double>(0,3);
    std::cout << distortionCoeff << std::endl;
    int step = 20;
    for(int v=0;v<=imageSize.height;v+=step){
        for(int u=0;u<=imageSize.width;u+=step){
            refPointsX.push_back((double)u);
            refPointsY.push_back((double)v);
        }
    }
    //歪補正
    std::vector<double> undistortedPointsX;
    std::vector<double> undistortedPointsY;
    double fx = intrinsic(0, 0);
    double fy = intrinsic(1, 1);
    double cx = intrinsic(0, 2);
    double cy = intrinsic(1, 2);
    double k1 = distortionCoeff(0);
    double k2 = distortionCoeff(1);
    double p1 = distortionCoeff(2);
    double p2 = distortionCoeff(3);
    for(int i=0,e=refPointsX.size();i<e;i++){
        double u = refPointsX[i];
        double v = refPointsY[i];

        double x1 = (u - cx)/fx;
        double y1 = (v - cy)/fy;

        double r = sqrt(x1*x1+y1*y1);

        double x2 = x1*(1.0+k1*r*r+k2*r*r*r*r)+2.0*p1*x1*y1+p2*(r*r+2.0*x1*x1);
        double y2 = y1*(1.0+k1*r*r+k2*r*r*r*r)+p1*(r*r+2.0*y1*y1)+2.0*p2*x1*y1;
        double mapx = x2*fx+cx;
        double mapy = y2*fy+cy;
        undistortedPointsX.push_back(mapx);
        undistortedPointsY.push_back(mapy);
    }
    //最適化
    printf("Before:\t%f,%f,%f,%f\r\n",distortionCoeff[0],distortionCoeff[1],distortionCoeff[2],distortionCoeff[3]);
    calc_invert_distortion_coeff functor2(distortionCoeff.size(),refPointsX.size(), undistortedPointsX, undistortedPointsY,
                                          refPointsX, refPointsY, intrinsic);

    NumericalDiff<calc_invert_distortion_coeff> numDiff2(functor2);
    LevenbergMarquardt<NumericalDiff<calc_invert_distortion_coeff> > lm2(numDiff2);
    int info = lm2.minimize(distortionCoeff);
    printf("After:\t%f,%f,%f,%f\r\n",distortionCoeff[0],distortionCoeff[1],distortionCoeff[2],distortionCoeff[3]);

    matInvDistort = (cv::Mat_<double>(1, 4) << distortionCoeff[0],distortionCoeff[1],distortionCoeff[2],distortionCoeff[3]);
}
