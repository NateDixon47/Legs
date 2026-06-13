#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "utilities/transforms.hpp"

double L1 = hypot(0.28154, 0.05165);
double L2 = hypot(0.29957, 0.01196);

double PHI1 = M_PI/2 + atan2(0.05165, 0.28154);
double PSI = 0.221341;

struct LegAngles
{
    double hip_yaw;
    double hip_pitch;
    double knee_pitch;
};


std::pair<double, double> planar_ik(double x, double y, bool elbow_up){
    double cos_knee_pitch = -(pow(L1, 2) + pow(L2, 2) - pow(x,2) - pow(y,2))/(2*L1*L2);
    double elbow_sign = elbow_up ? 1.0 : -1.0;

    double D = cos_knee_pitch;
    double C = sqrt(1-pow(D,2)) * elbow_sign;
    double q2 = atan2(C, D);

    double gamma = atan2(y,x);
    double cos_beta = (pow(L1, 2) + pow(x,2) + pow(y,2) - pow(L2,2))/(2*L1*sqrt(pow(x,2) + pow(y,2)));
    double B = cos_beta;

    double sin_beta = sqrt(1-pow(B,2)) * elbow_sign;
    double G = sin_beta;

    double BETA = atan2(G,B);
    double q1 = gamma - BETA;

    double hip_pitch = q1 + PHI1;
    double knee_pitch = PSI - q2;

    return {hip_pitch, knee_pitch};
}

LegAngles full_ik(Eigen::Vector3d p_body, bool elbow_up){
    Eigen::Isometry3d body_2_hip = form_TF_fixed(0.0, 1.5708, 0.0, 0.125, 0.02088, -0.0125);
    Eigen::Vector3d p_hip = body_2_hip.inverse() * p_body;

    // Natural (in-limits) yaw branch: atan2(z,y) + pi, wrapped to (-pi, pi].
    double hip_yaw = std::remainder(std::atan2(p_hip.z(), p_hip.y()) + M_PI, 2.0 * M_PI);

    // Undo the yaw rotation, then shift to the hip-pitch origin -> planar frame.
    Eigen::Vector3d p_planar = Eigen::AngleAxisd(-hip_yaw, Eigen::Vector3d::UnitX()) * p_hip
                               - Eigen::Vector3d(0.074, 0.0, 0.0);
    std::pair<double, double> p_ik = planar_ik(p_planar.x(), p_planar.y(), elbow_up);
    LegAngles angles;
    angles.hip_yaw = hip_yaw;
    angles.hip_pitch = p_ik.first;
    angles.knee_pitch = p_ik.second;

    return angles;
}




int main(){
    float x = 0.3;
    float y = 0.2;
    auto [hip_pitch, knee_pitch] = planar_ik(x, y, true);

    // std::cout << hip_pitch << " " << knee_pitch << std::endl;
    Eigen::Vector3d p(0.235, -0.400, -0.097);
    LegAngles angles = full_ik(p, false);
    std::cout << angles.hip_yaw << " " << angles.hip_pitch << " " << angles.knee_pitch << std::endl;
    return 0;
}