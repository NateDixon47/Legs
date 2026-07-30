#pragma once

#include <Eigen/Dense>
#include <array>
#include <vector>
#include <cmath>

namespace capturepoint {

class CapturePoint {
    public:
        CapturePoint() : K_(1.0),  height_(0.5){}

        Eigen::Vector2d compute_cp(const Eigen::Vector2d &x, const Eigen::Vector2d &x_dot) {
            return x + x_dot/w_;

        }

        Eigen::Vector2d compute_px(const Eigen::Vector2d &x, const Eigen::Vector2d &x_dot, Eigen::Vector2d x_ddot, Eigen::Vector2d x_dot_des) {
            Eigen::Vector2d xi = compute_cp(x, x_dot);
            Eigen::Vector2d xi_dot = x_dot + x_ddot/w_;
            Eigen::Vector2d xi_des  = x + x_dot_des/w_;

            return xi - (xi_dot/w_) + K_*(xi-xi_des);
        }

        Eigen::Vector3d swing_trajectory(double T, double t_swing, Eigen::Vector2d p_start, Eigen::Vector2d p_des, double step_height) {
            double s = std::min(t_swing / T, 1.0);

            // Horizontal position
            double px = p_start[0] + std::pow(s, 2) * (3-2*s) * (p_des[0] - p_start[0]);
            double py = p_start[1] + std::pow(s, 2) * (3-2*s) * (p_des[1] - p_start[1]);

            // Vertical position
            double pz = 4 * step_height * s * (1-s);

            return {px, py, pz};
        }

        Eigen::Vector2d predict_eos(Eigen::Vector2d xi, Eigen::Vector2d p_stance, double T, double t_swing) {
            return p_stance + (xi - p_stance) * std::exp(w_*(T-t_swing));
        }

        Eigen::Vector2d step_location(Eigen::Vector2d xi_eos, Eigen::Vector2d x_dot_des, double T) {
            double b_lat = leg_offset_ / (1.0 + std::exp(w_*T));
            return {xi_eos[0] - x_dot_des[0]/w_, xi_eos[1] + side_ * b_lat};
        }

        void set_height(double height) {
            height_ = height;
            w_ = std::sqrt(9.81/height_);
        }

        void switch_stance() {side_ = side_ * -1;}


    private:
        double height_;
        double w_ = std::sqrt(9.81/height_);
        float K_;
        double leg_offset_ = 0.1; // left and right leg offset for step
        int side_ = 1;

};

}