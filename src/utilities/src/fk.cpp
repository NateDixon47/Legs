#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include "utilities/transforms.hpp"


// Eigen::Matrix3d Rx(float roll){
//     Eigen::Matrix3d R;
//     R << 1, 0, 0,
//         0, std::cos(roll), -std::sin(roll),
//         0, std::sin(roll), std::cos(roll);
//     return R;
// }

// Eigen::Matrix3d Ry(float pitch){
//     Eigen::Matrix3d R;
//     R << std::cos(pitch), 0, std::sin(pitch),
//          0, 1, 0,
//          -std::sin(pitch), 0, std::cos(pitch);
//     return R;
// }

// Eigen::Matrix3d Rz(float yaw){
//     Eigen::Matrix3d R;
//     R << std::cos(yaw), -std::sin(yaw), 0,
//          std::sin(yaw), std::cos(yaw), 0,
//          0, 0, 1;
//     return R;
// }

// form_TF_fixed now lives in src/transforms.cpp (declared in utilities/transforms.hpp)

Eigen::Isometry3d form_TF_rotation(double q, int axis){
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d R;
    if (axis == 1) {R = Eigen::AngleAxisd(q, Eigen::Vector3d::UnitX()).toRotationMatrix();}
    else if (axis == 2) {R = Eigen::AngleAxisd(q, Eigen::Vector3d::UnitY()).toRotationMatrix();}
    else if (axis == 3) {R = Eigen::AngleAxisd(q, Eigen::Vector3d::UnitZ()).toRotationMatrix();}
    else {R = Eigen::Matrix3d::Identity();}

    T.linear() = R;
    T.translation() = Eigen::Vector3d::Zero();
    return T;
}

Eigen::Isometry3d TF(double roll, double pitch, double yaw, double x, double y, double z, double q, int axis){
    Eigen::Isometry3d T_fixed = form_TF_fixed(roll, pitch, yaw, x, y, z);
    Eigen::Isometry3d T_rot = form_TF_rotation(q, axis);

    return T_fixed * T_rot;
}

Eigen::Vector3d FK(double q_yaw, double q_pitch, double q_knee){
    Eigen::Isometry3d T_hip_yaw   = TF(0,       1.5708, 0,  0.125,   0.02088, -0.0125, q_yaw,   1);
    Eigen::Isometry3d T_hip_pitch = TF(1.5708,  0,     -1.5708, 0.074, 0.0,  0.0,  q_pitch, 2);
    Eigen::Isometry3d T_knee      = TF(3.14159, 0,      0,  0.28154, 0.0,    0.05165, q_knee,  2);
    Eigen::Isometry3d T_ankle     = form_TF_fixed(-1.5708, 1.10916, 0, 0.29957, 0.0, 0.01196);
    Eigen::Isometry3d T = T_hip_yaw * T_hip_pitch * T_knee * T_ankle;
    return T.translation();
}


int main(){
    double hip_yaw = 0.0;
    double hip_pitch = 0.0;
    double knee_pitch = 0.25;
    Eigen::Vector3d q = FK(hip_yaw, hip_pitch, knee_pitch);
    std::cout << q << std::endl;
    return 0;
}


