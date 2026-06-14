#include <chrono>
#include <functional>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "utilities/leg.hpp"
#include "utilities/robot.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

/*
This initial IK node will take in a foot position for the left leg in the body frame 
and publish the joint angles to move the foot to that position.
*/
class IK_Node : public rclcpp::Node{
    public:
        IK_Node() : Node("ik_node"), robot_() {
            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
            p_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("foot_pos", 10, std::bind(&IK_Node::ik_callback, this, std::placeholders::_1));
            js_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&IK_Node::fk_check, this, std::placeholders::_1));
        }

    private:
        robot::Robot robot_;


        void ik_callback(const std_msgs::msg::Float64MultiArray &msg) {
            if (msg.data.size() < 6){
                RCLCPP_WARN(this->get_logger(), "foot_pos needs 6 values: [left x,y,z, right x,y,z]");
                return;
            }
            Eigen::Vector3d p_left(msg.data[0], msg.data[1], msg.data[2]);
            Eigen::Vector3d p_right(msg.data[3], msg.data[4], msg.data[5]);

            auto cmd = robot_.generate_command(p_left, p_right);
            if (!cmd) {
                RCLCPP_WARN(this->get_logger(), "IK: No solution for one or both foot targets");
                return;
            }

            std_msgs::msg::Float64MultiArray output;
            
            output.data = std::vector<double>(cmd->data(), cmd->data() + cmd->size());
            publisher_->publish(output);

            last_target_ << p_left, p_right;     // remember what we commanded, for the FK check
            have_target_ = true;
        }

        // Look up a joint's position by name (JointState ordering is not guaranteed).
        static bool get_joint(const sensor_msgs::msg::JointState &msg,
                              const std::string &name, double &out) {
            for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
                if (msg.name[i] == name) { out = msg.position[i]; return true; }
            }
            return false;
        }

        void fk_check(const sensor_msgs::msg::JointState &msg){
            if (!have_target_) return;   // nothing commanded yet

            double yaw_l, pitch_l, knee_l, yaw_r, pitch_r, knee_r;
            if (!get_joint(msg, "left_hip_yaw",   yaw_l)   ||
                !get_joint(msg, "left_hip_pitch", pitch_l) ||
                !get_joint(msg, "left_knee",      knee_l)  ||
                !get_joint(msg, "right_hip_yaw",   yaw_r)   ||
                !get_joint(msg, "right_hip_pitch", pitch_r) ||
                !get_joint(msg, "right_knee",      knee_r)) {
                return;   // left-leg joints not present in this message
            }
            Eigen::Vector3d target_l = last_target_.segment<3>(0);
            Eigen::Vector3d target_r = last_target_.segment<3>(3);
            Eigen::Vector3d foot_l = robot_.left_leg_.FK(yaw_l, pitch_l, knee_l);
            Eigen::Vector3d foot_r = robot_.right_leg_.FK(yaw_r, pitch_r, knee_r);
            double err_mm_l = (foot_l - target_l).norm() * 1000.0;
            double err_mm_r = (foot_r - target_r).norm() * 1000.0;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "FK check: left err %.1f mm | right err %.1f mm", err_mm_l, err_mm_r);
        }

        Eigen::VectorXd last_target_ = Eigen::VectorXd::Zero(6);
        bool have_target_ = false;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr p_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_subscriber_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IK_Node>());
    rclcpp::shutdown();
    return 0;
}
