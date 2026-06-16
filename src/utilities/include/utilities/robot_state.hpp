#ifndef UTILITIES_ROBOT_STATE_HPP
#define UTILITIES_ROBOT_STATE_HPP

#include <array>
#include <Eigen/Dense>

namespace robot {

struct RobotState {
    Eigen::Isometry3d base_pose = Eigen::Isometry3d::Identity();
    Eigen::Vector3d base_lin_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d base_ang_vel = Eigen::Vector3d::Zero();

    // Actuated joints
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(6);

    // Foot contact
    std::array<bool, 2> foot_in_contact = {false, false};

    // Timestamp
    double stamp = 0.0;
};

}

#endif