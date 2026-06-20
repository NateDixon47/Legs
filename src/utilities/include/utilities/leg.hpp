#pragma once

// Single-leg kinematics for the bipedal robot.
// This header DECLARES the interface (what data + functions the class has).
// The implementation lives in leg.cpp. Pure C++/Eigen — no ROS dependencies.

#include <Eigen/Geometry>
#include <optional>
#include <utility>

namespace legs {

enum class Side { Left, Right };

struct LegAngles {
    double hip_yaw;
    double hip_pitch;
    double knee_pitch;
};

struct JointLimit {
    double min;
    double max;
    bool contains(double q) const { return q >= min && q <= max; }
};

struct LegLimit {
    JointLimit hip_yaw;
    JointLimit hip_pitch;
    JointLimit knee_pitch;
};

// One joint's fixed origin (from the URDF) plus its rotation axis.
struct JointOrigin {
    double roll, pitch, yaw;   // fixed rotation (rpy)
    double x, y, z;            // fixed translation (xyz)
    int axis;                  // 1=X, 2=Y, 3=Z, 0 = fixed (no joint rotation)
};

// Everything that differs between the left and right leg.
struct LegParams {
    JointOrigin hip_yaw;
    JointOrigin hip_pitch;
    JointOrigin knee_pitch;
    JointOrigin ankle;

    double PHI1;
    double PSI;
    double beta_sign;   // +1/-1: sign of BETA in the planar hip solve (flips for the mirrored leg)
    double L2;          // tibia length (right ankle differs slightly from left)

    LegLimit limits;
};


class Leg {
public:
    explicit Leg(Side side);

    // Foot position in the body (base) frame for the given joint angles.
    Eigen::Vector3d FK(double q_yaw, double q_pitch, double q_knee) const;

    // Joint angles that place the foot at p_body (body frame).
    // Returns std::nullopt if the target is unreachable or out of joint limits.
    std::optional<LegAngles> IK(Eigen::Vector3d p_body, bool elbow_up) const;

    LegAngles get_joint_angles() const;

    void set_joint_angles(LegAngles q);

    bool within_limits(LegAngles q) const;

    Eigen::Matrix3d jacobian();


private:
    // 2-link planar solver (hip pitch + knee). nullopt if unreachable.
    std::optional<std::pair<double, double>> planar_ik(double x, double y, bool elbow_up) const;

    double L1_;   // femur length (same both sides; L2 is per-side in LegParams)
    LegParams params_;
    LegAngles joint_angles {0, 0, 0};
};

}  // namespace legs
