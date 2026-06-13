#pragma once

#include <Eigen/Geometry>
#include <optional>
#include <utility>

// Single-leg kinematics for the bipedal robot.
// Pure C++/Eigen — no ROS dependencies. The implementation lives in leg.cpp.

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

private:
    // 2-link planar solver (hip pitch + knee). nullopt if unreachable.
    std::optional<std::pair<double, double>> planar_ik(double x, double y, bool elbow_up) const;

    Side side_;

    // Leg geometry constants — set in the constructor (see leg.cpp).
    // NOTE: FK joint origins and IK's body_2_hip are currently LEFT-leg only;
    // branch them on side_ when adding the mirrored right leg (roadmap #2).
    double L1_;
    double L2_;
    double PHI1_;
    double PSI_;

    LegLimit limits_;
    LegAngles joint_angles {0, 0, 0};
};
