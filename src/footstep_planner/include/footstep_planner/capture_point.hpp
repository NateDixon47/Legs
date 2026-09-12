#pragma once

#include <Eigen/Dense>
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>

namespace capturepoint {

class CapturePoint {
    public:
        CapturePoint() : K_(1.0, 1.0),  height_(0.475){}

        Eigen::Vector2d compute_cp(const Eigen::Vector2d &x, const Eigen::Vector2d &x_dot) {
            return x + x_dot/w_;

        }

        Eigen::Vector2d compute_px(const Eigen::Vector2d &x, const Eigen::Vector2d &x_dot, Eigen::Vector2d x_ddot, Eigen::Vector2d x_dot_des) {
            Eigen::Vector2d xi = compute_cp(x, x_dot);
            Eigen::Vector2d xi_dot = x_dot + x_ddot/w_;
            Eigen::Vector2d xi_des  = x + x_dot_des/w_;

            return xi - (xi_dot/w_) + K_.cwiseProduct(xi-xi_des);
        }

        // Adaptive sine swing-height profile (port of the reference sin_adapt).
        //   t   : time since liftoff        T  : swing duration
        //   hs  : target apex clearance     h0 : liftoff height rel. to the landing point
        // Rises from h0 to the apex at tau=0.5, then descends to 0 (touchdown) at tau=1.
        // h_m = max(1.1*h0, hs) guarantees real lift even on a big step down; with the
        // default h0=0 (flat ground) this reduces to a symmetric sine arc of height hs.
        Eigen::Vector2d swing_height(double t, double T, double hs, double h0 = 0.0) {
            double tau = std::clamp(t / T, 0.0, 1.0);
            double h_m = std::max(1.1 * h0, hs);
            double h_tau, vel_des;
            Eigen::Vector2d output;
            if (tau < 0.5) {
                h_tau = (h_m - h0) * std::sin(M_PI * tau) + h0;   // rise: h0 -> apex
                vel_des = (h_m - h0) * (M_PI / T) * std::cos(M_PI*tau);
            } else {
                h_tau = h_m * std::sin(M_PI * tau);               // descend: apex -> 0
                vel_des = h_m * (M_PI / T) * std::cos(M_PI*tau);
            }
            output = {h_tau, vel_des};
            return output;
        }

        // Eigen::Vector3d swing_trajectory(double T, double t_swing, Eigen::Vector2d p_start, Eigen::Vector2d p_des, double step_height) {
        //     // Horizontal: track the foothold directly from the start (no interpolation).
        //     double px = p_des[0];
        //     double py = p_des[1];

        //     // Vertical: adaptive sine profile; touchdown (pz=0) at t_swing = T.
        //     double pz = swing_height(t_swing, T, step_height);

        //     return {px, py, pz};
        // }

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

        double get_omega() { return w_; }

        void switch_stance() {side_ = side_ * -1;}


    private:
        double height_;
        double w_ = std::sqrt(9.81/height_);
        Eigen::Vector2d K_;
        double leg_offset_ = 0.25; // left and right leg offset for step
        int side_ = 1;

};

}