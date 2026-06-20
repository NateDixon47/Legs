#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <algorithm>
#include <vector>
#include <array>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "utilities/leg.hpp"
#include "utilities/robot.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "legs_control/trajectory_generator.hpp"

using namespace std::chrono_literals;

class Step_Node : public rclcpp::Node {
    public:
        Step_Node() : Node("step_node") {
            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/position_controller/commands", 10);
            p_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/foot_pos", 10, std::bind(&Step_Node::get_foot_pos, this, std::placeholders::_1));
            js_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Step_Node::get_state, this, std::placeholders::_1));
            timer_ = this->create_wall_timer(10ms, std::bind(&Step_Node::timer_callback, this));

            // Validate both home foot positions are reachable at startup.
            if (!robot_.generate_command(home_l_foot_, home_r_foot_)) {
                throw std::runtime_error("home foot position is unreachable");
            }
        }

    private:
        // Target: 6-element [lx,ly,lz, rx,ry,rz] in the base frame. Builds a Cartesian
        // swing trajectory for each foot; IK happens per-tick in timer_callback.
        void get_foot_pos(const std_msgs::msg::Float64MultiArray &msg) {
            if (!have_state_) {
                RCLCPP_WARN(this->get_logger(), "Did not receive joint states yet.");
                return;
            }
            Eigen::Vector3d target_l(msg.data[0], msg.data[1], msg.data[2]);
            Eigen::Vector3d target_r(msg.data[3], msg.data[4], msg.data[5]);

            // qi = current foot position (FK of measured joints) for each leg.
            Eigen::Vector3d qi_l = robot_.left_leg_.FK(q_[0], q_[1], q_[2]);
            Eigen::Vector3d qi_r = robot_.right_leg_.FK(q_[3], q_[4], q_[5]);
            Eigen::Vector3d v0 = Eigen::Vector3d::Zero();

            // Store Cartesian foot trajectories; generate_command IKs them each tick.
            foot_traj_l_ = generate_trajectory(qi_l, target_l, v0, v0, T_step_, dt_traj_);
            foot_traj_r_ = generate_trajectory(qi_r, target_r, v0, v0, T_step_, dt_traj_);

            t_step_start_ = this->now().seconds();
            have_foot_pos_ = true;
        }

        void get_state(const sensor_msgs::msg::JointState::SharedPtr msg) {
            // All 6 joints, mapped by name into joint_names_ order (left 3, right 3).
            static const std::array<std::string, 6> joint_names = {
                "left_hip_yaw", "left_hip_pitch", "left_knee",
                "right_hip_yaw", "right_hip_pitch", "right_knee"};
            for (size_t i = 0; i < joint_names.size(); ++i) {
                auto it = std::find(msg->name.begin(), msg->name.end(), joint_names[i]);
                if (it == msg->name.end()) {
                    return;  // a joint is missing this message; try again next one
                }
                size_t idx = std::distance(msg->name.begin(), it);
                q_[static_cast<int>(i)] = msg->position[idx];
            }
            have_state_ = true;
        }

        void timer_callback() {
            if (!have_state_) { return; }

            // Current desired foot positions: home until a target, then the trajectory
            // sampled by elapsed time (clamped at the end).
            Eigen::Vector3d foot_l, foot_r;
            if (!have_foot_pos_) {
                foot_l = home_l_foot_;
                foot_r = home_r_foot_;
            } else {
                double elapsed = this->now().seconds() - t_step_start_;
                int k = static_cast<int>(std::floor(elapsed / dt_traj_));
                k = std::min(k, static_cast<int>(foot_traj_l_.size()) - 1);
                foot_l = foot_traj_l_[k];
                foot_r = foot_traj_r_[k];
            }

            // Both legs' IK in one call -> 6 joint angles (left 3, right 3).
            std::optional<Eigen::VectorXd> q_des = robot_.generate_command(foot_l, foot_r);
            if (!q_des) {
                RCLCPP_WARN(this->get_logger(), "generate_command unreachable; holding last");
                return;
            }
            publish_positions(*q_des);
        }

        void publish_positions(const Eigen::VectorXd &q) {
            std_msgs::msg::Float64MultiArray cmd;
            cmd.data = {q[0], q[1], q[2], q[3], q[4], q[5]};
            publisher_->publish(cmd);
        }

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr p_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;

        robot::Robot robot_;

        // Measured joint angles, all 6 (left_hip_yaw, left_hip_pitch, left_knee, right_...).
        Eigen::Matrix<double, 6, 1> q_ = Eigen::Matrix<double, 6, 1>::Zero();

        // Home/idle foot positions (base frame), one per leg.
        Eigen::Vector3d home_l_foot_{0.125, -0.475, -0.1};
        Eigen::Vector3d home_r_foot_{-0.125, -0.475, -0.1};

        // Cartesian swing trajectories (foot positions), one per leg.
        std::vector<Eigen::Vector3d> foot_traj_l_;
        std::vector<Eigen::Vector3d> foot_traj_r_;

        bool have_state_ = false;
        bool have_foot_pos_ = false;
        double t_step_start_ = 0.0;
        double dt_traj_ = 0.01;   // = control period (100 Hz)
        double T_step_ = 0.5;     // swing duration
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Step_Node>());
    rclcpp::shutdown();
    return 0;
}
