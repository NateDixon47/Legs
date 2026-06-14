#pragma once
#include <array>
#include <string>
#include <optional>
#include <Eigen/Dense>
#include "utilities/leg.hpp"

/*
Robot Class: Acts as the whole leg pair composer. It holds both legs and joint ordering and it
translates between the 2 languages of the system.
- per-leg angle view (LegAngles)
- the flat 6-number used by the ROS controller

What it holds:
- left and right leg instances
- joint ordering: six joint names in the order the controller expects
*/

namespace robot {

class Robot {
    public:
        Robot();

        // Takes a desired foot position for each leg, runs both leg IK's and
        // assembles the output into a single ordered command vector.
        std::optional<Eigen::VectorXd> generate_command(Eigen::Vector3d left_foot, Eigen::Vector3d right_foot);
        legs::Leg left_leg_;
        legs::Leg right_leg_;
    private:
        std::array<std::string, 6> joint_names_;
};

}  // namespace robot
