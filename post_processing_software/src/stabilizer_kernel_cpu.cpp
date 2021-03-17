#include <Eigen/Dense>
#include <vector>
Eigen::Vector2d warp_undistort(
    Eigen::Vector2d p,                              // UV coordinate position in a image.
    double zoom_ratio,
    std::vector<Eigen::Matrix3d> rotation_matrix,   // Rotation Matrix in each rows.
    Eigen::VectorXd d,                              // Distortion parameters.
    Eigen::Vector2d f, Eigen::Vector2d c)
{
    double &k1 = d[0]; 
    double &k2 = d[1]; 
    double &p1 = d[2]; 
    double &p2 = d[3];  
    Eigen::Vector2d x1 = (p - c).array() / f.array();
    double r2 = x1.squaredNorm();
    Eigen::Vector2d x2 = x1 * (1.f + k1 * r2 + k2 * r2 * r2);
    x2 += Eigen::Vector2d(2.f * p1 * x1[0] * x1[1] + p2 * (r2 + 2.f * x1[0] * x1[0]), p1 * (r2 + 2.f * x1[1] * x1[1]) + 2.f * p2 * x1[0] * x1[1]);

    Eigen::Vector3d x3 = Eigen::Vector3d(x2[0], x2[1], 1.0);
    Eigen::Matrix3d R = rotation_matrix[(int)round(p[1])];
    Eigen::Vector3d XYZ = R * x3;
    x2 = Eigen::Vector2d(XYZ[0], XYZ[1]).array() / XYZ[2];
    return x2.array() * f.array() * zoom_ratio + c.array();
}



Eigen::Vector2d warp_optical_flow(
    Eigen::Vector2d p,              // UV Coordinate in an image
    cv::Mat optical_flow
)
{
    
    p + optical_flow
}