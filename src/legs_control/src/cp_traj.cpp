#include <Eigen/Dense>


// 2D swing trajectory using cubic interpolation and parabolic arc
std::array<double, 2> swing_trajectory(double T, double t_swing, Eigen::VectorXd p_start, Eigen::VectorXd p_des, double step_height) {
    double s = t_swing/T;
    double px = p_start[0] + std::pow(s,2) * (3-2*s) * (p_des[0]-p_start[0]);
    double pz = step_height * s * (1-s);

    return {px, pz};

}