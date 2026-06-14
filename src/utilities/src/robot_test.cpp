// Standalone test for the Robot class (Option B build — no CMake / no ROS).
// Run from  ~/legs_ws/src/utilities :
//
//   g++ src/robot_test.cpp src/robot.cpp src/leg.cpp src/transforms.cpp \
-I include -I /usr/include/eigen3 -std=c++17 -o /tmp/robot_test && /tmp/robot_test

#include "utilities/robot.hpp"
#include <cstdio>

static void run_case(robot::Robot& r, const char* name,
                     Eigen::Vector3d left_foot, Eigen::Vector3d right_foot) {
    auto cmd = r.generate_command(left_foot, right_foot);
    if (cmd) {
        std::printf("%-18s SOLVED  q = [", name);
        for (int i = 0; i < cmd->size(); ++i) std::printf(" %+.3f", (*cmd)(i));
        std::printf(" ]\n");
    } else {
        std::printf("%-18s nullopt (no solution)\n", name);
    }
}

int main() {
    robot::Robot r;

    // Both feet reachable -> expect a 6-vector [Lyaw,Lpitch,Lknee, Ryaw,Rpitch,Rknee]
    run_case(r, "both reachable",
             Eigen::Vector3d(0.1250, -0.5196, -0.0430),
             Eigen::Vector3d(-0.1250, -0.5198, -0.0429));

    // One foot far out of reach -> expect nullopt
    run_case(r, "left unreachable",
             Eigen::Vector3d(10.0, 10.0, 10.0),
             Eigen::Vector3d(-0.1250, -0.5198, -0.0429));

    run_case(r, "right unreachable",
             Eigen::Vector3d(0.1250, -0.5196, -0.0430),
             Eigen::Vector3d(10.0, 10.0, 10.0));

    return 0;
}
