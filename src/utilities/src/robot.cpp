#include "utilities/robot.hpp"

namespace robot {

Robot::Robot()
    : left_leg_(legs::Side::Left),
      right_leg_(legs::Side::Right),
      joint_names_{"left_hip_yaw", "left_hip_pitch", "left_knee",
                   "right_hip_yaw", "right_hip_pitch", "right_knee"} {}

std::optional<Eigen::VectorXd> Robot::generate_command(Eigen::Vector3d left_foot, Eigen::Vector3d right_foot) {
    Eigen::VectorXd q(6);
    auto sol_l = left_leg_.IK(left_foot, false);
    auto sol_r = right_leg_.IK(right_foot, false);

    if (!sol_l || !sol_r) { return std::nullopt; }

    q << sol_l->hip_yaw, sol_l->hip_pitch, sol_l->knee_pitch,
         sol_r->hip_yaw, sol_r->hip_pitch, sol_r->knee_pitch;

    return q;
}

}  // namespace robot