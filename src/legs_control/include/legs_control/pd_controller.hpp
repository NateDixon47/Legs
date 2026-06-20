#pragma once

#include <Eigen/Dense>
#include <array>
#include <vector>
#include <cmath>

class PD_Controller {
    public:
        PD_Controller(Eigen::Vector3d Kp, Eigen::Vector3d Kd) : Kp_(Kp), Kd_(Kd) {}

        Eigen::Vector3d compute(Eigen::Vector3d q, Eigen::Vector3d q_des, Eigen::Vector3d q_dot, Eigen::Vector3d q_dot_des) {
            Eigen::Vector3d tau = Kp_.cwiseProduct(q_des - q) + Kd_.cwiseProduct(q_dot_des - q_dot);
            return tau.cwiseMin(-tau_limit_).cwiseMax(tau_limit_);
        }

    private:
        Eigen::Vector3d Kp_;
        Eigen::Vector3d Kd_;
        float tau_limit_ = 60.0;

};