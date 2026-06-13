#pragma once
#include <Eigen/Geometry>

Eigen::Isometry3d form_TF_fixed(double roll, double pitch, double yaw, double x, double y, double z);