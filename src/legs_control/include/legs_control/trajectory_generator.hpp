#pragma once

#include <Eigen/Dense>
#include <array>
#include <vector>
#include <cmath>

// Returns {a0, a1, a2, a3} for the cubic q(t) = a0 + a1 t + a2 t^2 + a3 t^3
// satisfying q(0)=q0, q'(0)=v0, q(T)=qf, q'(T)=vf.
inline std::array<Eigen::Vector3d, 4> cubic_coeffs(
    const Eigen::Vector3d& q0, const Eigen::Vector3d& v0,
    const Eigen::Vector3d& qf, const Eigen::Vector3d& vf, double T) {
    Eigen::Vector3d a0 = q0;
    Eigen::Vector3d a1 = v0;
    Eigen::Vector3d a2 = (3 * (qf - q0) - (2 * v0 + vf) * T) / (T * T);
    Eigen::Vector3d a3 = (-2 * (qf - q0) + (v0 + vf) * T) / (T * T * T);
    return {a0, a1, a2, a3};
}

inline Eigen::Vector3d evaluate_q(double t,
    const Eigen::Vector3d& a0, const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2, const Eigen::Vector3d& a3) {
    return a0 + a1 * t + a2 * t * t + a3 * t * t * t;
}

// Builds a swing-foot trajectory that lifts from qi, passes through an apex,
// and lands at qf. Returns positions sampled every dt over total duration T.
inline std::vector<Eigen::Vector3d> generate_trajectory(
    Eigen::Vector3d qi, Eigen::Vector3d qf,
    Eigen::Vector3d vi, Eigen::Vector3d vf, double T, double dt) {

    // In this leg's base frame the vertical axis is Y (the foot hangs in -Y), so
    // "lifting" the swing foot means raising the Y component, not Z. step_height is
    // the clearance above the higher of the two endpoints (+Y is up here).
    double step_height = 0.1;
    double apex_y = std::max(qi[1], qf[1]) + step_height;
    Eigen::Vector3d apex((qi[0] + qf[0]) / 2.0,   // X: midpoint (forward/back)
                         apex_y,                  // Y: lifted (up)
                         (qi[2] + qf[2]) / 2.0);  // Z: midpoint (sideways)

    // Velocity at the apex via point: the secant across the whole step
    // (Catmull-Rom). Smooth horizontal motion, ~zero vertical at the top on
    // level ground. Set to Zero() to momentarily stop the foot at the apex.
    Eigen::Vector3d v_apex = (qf - qi) / T;

    double Th = T / 2.0;                              // each segment is half the step
    int n = static_cast<int>(std::round(Th / dt));   // samples per segment

    std::vector<Eigen::Vector3d> trajectory;
    trajectory.reserve(2 * n + 1);

    // --- Segment 1: qi -> apex over local time [0, Th] ---
    // Structured binding (C++17): unpacks the std::array into named coeffs.
    auto [a0, a1, a2, a3] = cubic_coeffs(qi, vi, apex, v_apex, Th);
    // i = 0..n-1: include qi, EXCLUDE the apex (segment 2 emits it) so the
    // join point isn't duplicated.
    for (int i = 0; i < n; ++i) {
        double t = i * dt;
        trajectory.push_back(evaluate_q(t, a0, a1, a2, a3));
    }

    // --- Segment 2: apex -> qf over local time [0, Th] ---
    auto [b0, b1, b2, b3] = cubic_coeffs(apex, v_apex, qf, vf, Th);
    // i = 0..n: emits the apex (i=0) and the final qf (i=n).
    for (int i = 0; i <= n; ++i) {
        double t = i * dt;
        trajectory.push_back(evaluate_q(t, b0, b1, b2, b3));
    }

    return trajectory;
}
