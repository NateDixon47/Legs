#include <chrono>
#include <functional>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "utilities/leg.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

/*
This initial IK node will take in a foot position for the left leg in the body frame 
and publish the joint angles to move the foot to that position.
*/
class IK_Node : public rclcpp::Node{
    public:
        IK_Node() : Node("ik_node"), left_leg_(legs::Side::Left) {
            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
            p_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("foot_pos", 10, std::bind(&IK_Node::ik_callback, this, std::placeholders::_1));
            js_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&IK_Node::fk_check, this, std::placeholders::_1));
        }

    private:
        legs::Leg left_leg_;

        void ik_callback(const std_msgs::msg::Float64MultiArray &msg) {
            if (msg.data.size() < 3){
                RCLCPP_WARN(this->get_logger(), "Foot position needs an x, y and z value");
                return;
            }
            Eigen::Vector3d p(msg.data[0], msg.data[1], msg.data[2]);

            auto sol = left_leg_.IK(p, false);   // false = natural knee-forward, in-limits branch
            if (!sol) {
                RCLCPP_WARN(this->get_logger(), "IK: No solution for that foot position");
                return;
            }

            std_msgs::msg::Float64MultiArray cmd;
            cmd.data = {sol->hip_yaw, sol->hip_pitch, sol->knee_pitch,
                        0.0, 0.0, 0.0};   // right leg held at 0 (controller spans all 6 joints)
            publisher_->publish(cmd);

            last_target_ = p;     // remember what we commanded, for the FK check
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

            double yaw, pitch, knee;
            if (!get_joint(msg, "left_hip_yaw",   yaw)   ||
                !get_joint(msg, "left_hip_pitch", pitch) ||
                !get_joint(msg, "left_knee",      knee)) {
                return;   // left-leg joints not present in this message
            }

            Eigen::Vector3d foot = left_leg_.FK(yaw, pitch, knee);
            double err_mm = (foot - last_target_).norm() * 1000.0;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "FK check: target (%.3f, %.3f, %.3f) | foot (%.3f, %.3f, %.3f) | error %.1f mm",
                last_target_.x(), last_target_.y(), last_target_.z(),
                foot.x(), foot.y(), foot.z(), err_mm);
        }

        Eigen::Vector3d last_target_ {0, 0, 0};
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
