#include "utilities/transforms.hpp"
#include <cmath>

Eigen::Isometry3d form_TF_fixed(double roll, double pitch, double yaw, double x, double y, double z){
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
    T.translation()= Eigen::Vector3d(x, y, z);
    return T;
}