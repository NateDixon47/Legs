#include <chrono>
#include <functional>
#include <string>
#include <memory>
#include <optional>
#include <algorithm>
#include <vector>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "utilities/leg.hpp"
#include "utilities/robot.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "legs_control/trajectory_generator.hpp"
#include "legs_control/pd_controller.hpp"

/*


*/
using namespace std::chrono_literals;

class Step_Node : public rclcpp::Node {
    public:
        Step_Node() : Node("step_node") {
            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
            p_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/foot_pos", 10, std::bind(&Step_Node::get_foot_pos, this, std::placeholders::_1));
            js_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&Step_Node::get_state, this, std::placeholders::_1));
            timer_ = this->create_wall_timer(10ms, std::bind(&Step_Node::timer_callback, this));

            // Precompute the home joint angles once (held whenever no target is active).
            std::optional<legs::LegAngles> home = left_leg_.IK(home_foot_, false);
            if (!home) {
                throw std::runtime_error("home foot position is unreachable");
            }
            q_home_ = Eigen::Vector3d(home->hip_yaw, home->hip_pitch, home->knee_pitch);
        }


    private:

        void get_foot_pos(const std_msgs::msg::Float64MultiArray &msg) {
            if (!have_state_) {
                RCLCPP_WARN(this->get_logger(), "Did not receive joint states yet.");
                return;  // need measured joints before we can FK the start point
            }
            foot_pos_ = {msg.data[0], msg.data[1], msg.data[2]};

            // qi = current foot position (FK of the measured joints), in the base frame.
            // vi = vf = 0: the foot starts and ends the swing at rest.
            Eigen::Vector3d qi = left_leg_.FK(q_[0], q_[1], q_[2]);
            Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
            Eigen::Vector3d vf = Eigen::Vector3d::Zero();

            std::vector<Eigen::Vector3d> foot_traj =
                generate_trajectory(qi, foot_pos_, v0, vf, 0.5, dt_traj_);

            // IK every point into a temporary, so a single unreachable point doesn't
            // leave q_traj_ half-updated. Commit only if the whole swing is reachable.
            std::vector<Eigen::Vector3d> q_traj;
            q_traj.reserve(foot_traj.size());
            for (const Eigen::Vector3d &p : foot_traj) {
                std::optional<legs::LegAngles> angles = left_leg_.IK(p, false);
                if (!angles) {
                    RCLCPP_WARN(this->get_logger(),
                                "IK unreachable along trajectory; ignoring this target");
                    return;  // keep the previous q_traj_
                }
                // LegAngles -> Vector3d in the same (hip_yaw, hip_pitch, knee) order as q_.
                q_traj.push_back(Eigen::Vector3d(
                    angles->hip_yaw, angles->hip_pitch, angles->knee_pitch));
            }

            q_traj_ = std::move(q_traj);              // commit the new joint-space trajectory
            t_step_start_ = this->now().seconds();    // anchor the timer's playback to now
            have_foot_pos_ = true;
        }

        void get_state(const sensor_msgs::msg::JointState::SharedPtr msg) {
            static const std::array<std::string, 3> joint_names = {
                "left_hip_yaw", "left_hip_pitch", "left_knee"
            };
            for (size_t i = 0; i < joint_names.size(); ++i) {
                auto it = std::find(msg->name.begin(), msg->name.end(), joint_names[i]);
                if (it == msg->name.end()){
                    return;
                }
            
            size_t idx = std::distance(msg->name.begin(), it);
            q_[i] = msg->position[idx];
            q_dot_[i] = msg->velocity[idx];
            }

            have_state_ = true;

        }

        void timer_callback() {
            if (!have_state_) { return; }   // no measured joints yet

            // Desired LEFT-leg joint angles: home pose until a target is commanded,
            // then the IK trajectory sampled by elapsed time.
            Eigen::Vector3d q_des_left;
            if (!have_foot_pos_) {
                q_des_left = q_home_;
            } else {
                double time = this->now().seconds() - t_step_start_;
                int k = static_cast<int>(std::floor(time / dt_traj_));
                k = std::min(k, static_cast<int>(q_traj_.size()) - 1);
                q_des_left = q_traj_[k];
            }

            // Position control: publish joint ANGLES. The MuJoCo servo holds them.
            // Right leg held at a fixed pose (zeros) so it doesn't move.
            publish_positions(q_des_left, Eigen::Vector3d::Zero());
        }

        // Build the 6-joint POSITION command (left 3 + right 3, in joint_names_ order).
        void publish_positions(const Eigen::Vector3d &q_left, const Eigen::Vector3d &q_right) {
            std_msgs::msg::Float64MultiArray cmd;
            cmd.data = {q_left[0], q_left[1], q_left[2],
                        q_right[0], q_right[1], q_right[2]};
            publisher_->publish(cmd);
        }

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr p_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_subscriber_;
        Eigen::Vector3d Kp_{150.0, 150.0, 150.0};
        Eigen::Vector3d Kd_{8.0, 8.0, 8.0};
        PD_Controller pd_{Kp_, Kd_};
        Eigen::Vector3d q_{0.0, 0.0, 0.0};
        Eigen::Vector3d q_dot_{0.0, 0.0, 0.0};
        Eigen::Vector3d foot_pos_{0.0, 0.0, 0.0};
        Eigen::Vector3d home_foot_{0.125, -0.475, -0.1};   // home/idle foot position (base frame)
        Eigen::Vector3d q_home_{0.0, 0.0, 0.0};       // home joint angles (IK of home_foot_)
        bool have_state_ = false;
        bool have_foot_pos_ = false;
        rclcpp::TimerBase::SharedPtr timer_;
        legs::Leg left_leg_{legs::Side::Left};
        std::vector<Eigen::Vector3d> q_traj_;
        double t_step_start_ = 0.0;
        double dt_traj_ = 0.01;  // = control period (100 Hz) → one trajectory point per tick

};



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Step_Node>());
    rclcpp::shutdown();
    return 0;
}