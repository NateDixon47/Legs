#include "utilities/leg.hpp"
#include "utilities/transforms.hpp"
#include <cmath>

namespace legs {
namespace {

Eigen::Isometry3d form_TF_rotation(double q, int axis){
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    if (axis == 1)      { T.linear() = Eigen::AngleAxisd(q, Eigen::Vector3d::UnitX()).toRotationMatrix(); }
    else if (axis == 2) { T.linear() = Eigen::AngleAxisd(q, Eigen::Vector3d::UnitY()).toRotationMatrix(); }
    else if (axis == 3) { T.linear() = Eigen::AngleAxisd(q, Eigen::Vector3d::UnitZ()).toRotationMatrix(); }
    return T;  // axis 0 -> identity rotation
}

Eigen::Isometry3d TF(double roll, double pitch, double yaw,
                     double x, double y, double z, double q, int axis){
    return form_TF_fixed(roll, pitch, yaw, x, y, z) * form_TF_rotation(q, axis);
}

// Build the fixed origin + joint rotation transform for a joint from its params.
Eigen::Isometry3d TF(const JointOrigin& o, double q){
    return TF(o.roll, o.pitch, o.yaw, o.x, o.y, o.z, q, o.axis);
}

// Just the fixed origin transform (no joint rotation).
Eigen::Isometry3d fixed_tf(const JointOrigin& o){
    return form_TF_fixed(o.roll, o.pitch, o.yaw, o.x, o.y, o.z);
}

LegParams params_for(Side side){
    if (side == Side::Left){
        return LegParams{
            // {roll,    pitch,   yaw,      x,       y,        z,       axis}
            /*hip_yaw   */ {0.0,     1.5708,  0.0,     0.125,   0.02088, -0.0125,  1},
            /*hip_pitch */ {1.5708,  0.0,    -1.5708,  0.074,   0.0,      0.0,     2},
            /*knee_pitch*/ {3.14159, 0.0,     0.0,     0.28154, 0.0,      0.05165, 2},
            /*ankle     */ {-1.5708, 1.10916, 0.0,     0.29957, 0.0,      0.01196, 0},
            /*PHI1*/ M_PI / 2 + std::atan2(0.05165, 0.28154),
            /*PSI */ 0.221341,
            /*limits*/ {{-1.57, 0.5}, {-0.18, 2.0}, {0.0, 2.57}}
        };
    } else {  // Side::Right — mirrored origins from the URDF
        // TODO (hardware): re-check the knee limit sign — the knee is belt-driven,
        // so the sim convention may not match the physical hardware. Verify on hardware.
        return LegParams{
            /*hip_yaw   */ {3.14159, 1.5708,  0.0,    -0.125,   0.02088, -0.0125,  1},
            /*hip_pitch */ {1.5708,  0.0,     1.5708,  0.074,   0.0,      0.0,     2},
            /*knee_pitch*/ {0.0,     0.0,     0.0,     0.28154, 0.0,     -0.05165, 2},
            /*ankle     */ {-1.5708, 1.10916, 0.0,     0.29977, 0.0,      0.01196, 0},
            /*PHI1*/ 0.0,   // PLACEHOLDER — DERIVE by probing right-leg FK (not 0, not the left value)
            /*PSI */ 0.0,   // PLACEHOLDER — DERIVE by probing right-leg FK
            /*limits*/ {{-0.5, 1.57}, {-2.0, 0.18}, {0.0, 2.57}}
        };
    }
}

}  // namespace


Leg::Leg(Side side)
    : L1_(std::hypot(0.28154, 0.05165)),
      L2_(std::hypot(0.29957, 0.01196)),
      params_(params_for(side)) {}

Eigen::Vector3d Leg::FK(double q_yaw, double q_pitch, double q_knee) const {
    Eigen::Isometry3d T = TF(params_.hip_yaw,    q_yaw)
                        * TF(params_.hip_pitch,  q_pitch)
                        * TF(params_.knee_pitch, q_knee)
                        * TF(params_.ankle,      0.0);   // ankle is fixed (axis 0)
    return T.translation();
}

std::optional<LegAngles> Leg::IK(Eigen::Vector3d p_body, bool elbow_up) const {
    Eigen::Isometry3d body_2_hip = fixed_tf(params_.hip_yaw);
    Eigen::Vector3d p_hip = body_2_hip.inverse() * p_body;

    // Natural (in-limits) yaw branch: atan2(z,y) + pi, wrapped to (-pi, pi].
    double hip_yaw = std::remainder(std::atan2(p_hip.z(), p_hip.y()) + M_PI, 2.0 * M_PI);

    // Undo the yaw rotation, then shift to the hip-pitch origin -> planar frame.
    Eigen::Vector3d hip_pitch_offset(params_.hip_pitch.x, params_.hip_pitch.y, params_.hip_pitch.z);
    Eigen::Vector3d p_planar = Eigen::AngleAxisd(-hip_yaw, Eigen::Vector3d::UnitX()) * p_hip
                             - hip_pitch_offset;

    auto p_ik = planar_ik(p_planar.x(), p_planar.y(), elbow_up);
    if (!p_ik) {
        return std::nullopt;
    }

    LegAngles angles;
    angles.hip_yaw    = hip_yaw;
    angles.hip_pitch  = p_ik->first;
    angles.knee_pitch = p_ik->second;

    if (!within_limits(angles)) {
        return std::nullopt;
    }
    return angles;
}

LegAngles Leg::get_joint_angles() const {
    return joint_angles;
}

void Leg::set_joint_angles(LegAngles q) {
    joint_angles = q;
}

bool Leg::within_limits(LegAngles q) const {
    return params_.limits.hip_yaw.contains(q.hip_yaw)
        && params_.limits.hip_pitch.contains(q.hip_pitch)
        && params_.limits.knee_pitch.contains(q.knee_pitch);
}

std::optional<std::pair<double, double>> Leg::planar_ik(double x, double y, bool elbow_up) const {
    double r = std::hypot(x, y);
    if (r > L1_ + L2_ || r < std::abs(L1_ - L2_)) {
        return std::nullopt;
    }

    double elbow_sign = elbow_up ? 1.0 : -1.0;

    double D = -(std::pow(L1_, 2) + std::pow(L2_, 2) - std::pow(x, 2) - std::pow(y, 2)) / (2 * L1_ * L2_);
    double C = std::sqrt(1 - std::pow(D, 2)) * elbow_sign;
    double q2 = std::atan2(C, D);

    double gamma = std::atan2(y, x);
    double B = (std::pow(L1_, 2) + std::pow(x, 2) + std::pow(y, 2) - std::pow(L2_, 2))
               / (2 * L1_ * std::sqrt(std::pow(x, 2) + std::pow(y, 2)));
    double G = std::sqrt(1 - std::pow(B, 2)) * elbow_sign;
    double BETA = std::atan2(G, B);
    double q1 = gamma - BETA;

    return std::make_pair(q1 + params_.PHI1, params_.PSI - q2);
}

}  // namespace legs
