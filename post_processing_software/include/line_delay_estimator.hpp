#ifndef __LINE_DELAY_ESTIMATOR_HPP__
#define __LINE_DERAY_ESTIMATOR_HPP__

#include <iostream>
#include <memory>
#include "Eigen/Dense"
#include "unsupported/Eigen/NonLinearOptimization"
#include "unsupported/Eigen/NumericalDiff"
#include <opencv2/opencv.hpp>
using namespace Eigen;

// Generic functor
template <typename _Scalar, int NX = Dynamic, int NY = Dynamic>
struct LDEFunctor
{
  typedef _Scalar Scalar;
  enum
  {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Matrix<Scalar, InputsAtCompileTime, 1> InputType;
  typedef Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
  typedef Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;
};

struct line_delay_functor : LDEFunctor<double>
{
  line_delay_functor(int inputs, int values,
                     std::shared_ptr<CameraInformation> &camera_info,
                     std::vector<cv::Point3d> &world_points,
                     std::map<int, std::vector<cv::Point2d>> &corner_dict,
                     VirtualGimbalManager &manager)
      : inputs_(inputs), values_(values),
        camera_info(camera_info), world_points(world_points),
        corner_dict(corner_dict),
        manager(manager)
  {
    camera_matrix = (cv::Mat_<float>(3, 3) << camera_info->fx_, 0, camera_info->cx_, 0, camera_info->fy_, camera_info->cy_, 0, 0, 1);
    dist_coeffs = (cv::Mat_<float>(1, 4) << camera_info->k1_, camera_info->k2_, camera_info->p1_, camera_info->p2_);
  }

  // Fixed parameters
  const int inputs_;
  const int values_;
  std::shared_ptr<CameraInformation> camera_info;
  std::vector<cv::Point3d> world_points;
  std::map<int, std::vector<cv::Point2d>> corner_dict;
  VirtualGimbalManager &manager;

  // Variables
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;

  int operator()(const VectorXd &b, VectorXd &fvec) const
  {
    double time_offset = b[0];
    double line_delay = b[1];
    printf("time_offset:%16.15f line_delay:%16.15f\n", time_offset, line_delay);
    // for (int i = 0; i < values_; ++i)
    // {
    //   fvec[i] = b[0] * (1.0 - exp(-b[1] * x[i])) - y[i];
    // }
    // return 0;

    //データが存在する全フレーム繰り返し
    std::map<int, std::vector<cv::Point2d>> undistorted_corner_dict;
    for (auto &el : corner_dict)
    {
      // undistorted_corner_dictを生成
      std::vector<cv::Point2d> dst;
      manager.getUndistortUnrollingChessBoardPoints(time_offset, el, dst, line_delay);
      undistorted_corner_dict[el.first] = dst;

      // if (0 == el.first)
      // {
      //   // std::cout << dst[0] << std::endl;
      //   printf("dst: %1.17f %1.17f\n",dst[0].x,dst[1].y);
      // }
    }

    // Variables
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    std::vector<std::vector<cv::Point3d>> vec_world_points;
    std::vector<std::vector<cv::Point2d>> vec_image_points;

    std::vector<double> per_image_errors;

    for (auto &el : undistorted_corner_dict)
    {
      cv::Mat rvec;
      cv::Mat tvec;

      cv::solvePnP(world_points, el.second, camera_matrix, dist_coeffs, rvec, tvec);
      rvecs.push_back(rvec);
      tvecs.push_back(tvec);
      vec_world_points.push_back(world_points);
      vec_image_points.push_back(el.second);
    }
    // double retval = manager.computeReprojectionErrors(vec_world_points, vec_image_points, rvecs, tvecs, camera_matrix, dist_coeffs, per_image_errors);
    manager.computeReprojectionErrors(vec_world_points, vec_image_points, rvecs, tvecs, camera_matrix, dist_coeffs, per_image_errors);
    // printf("retval:%17.16f\n",retval);

    fvec = Eigen::Map<Eigen::VectorXd>(&per_image_errors[0], per_image_errors.size());
    // for(int i=0;i<per_image_errors.size();++i){
    //   fvec[i] = per_image_errors[i];
    // }


    // std::cout << fvec.transpose() << std::endl << std::flush;
    return 0;
  }
  /*
    int df(const VectorXd& b, MatrixXd& fjac)
    {	
        for (int i = 0; i < values_; ++i) {
	  fjac(i, 0) = (1.0 - exp(-b[1]*x[i]));
	  fjac(i, 1) = (b[0]*x[i] * exp(-b[1]*x[i]));
        }
        return 0;
    }
  */

  int inputs() const { return inputs_; }
  int values() const { return values_; }
};

#endif //__LINE_DERAY_ESTIMATOR_HPP__
