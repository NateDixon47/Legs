#include <chrono>
#include <functional>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "utilities/leg.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

/*
This initial IK node will take in a foot position for the left leg in the body frame 
and publish the joint angles to move the foot to that position.
*/
class IK_Node : public rclcpp::Node{
    public:
        IK_Node() : Node("ik_node"), left_leg_(legs::Side::Left) {
            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
            subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("foot_pos", 10, std::bind(&IK_Node::ik_callback, this, std::placeholders::_1));
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
        }
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IK_Node>());
    rclcpp::shutdown();
    return 0;
}
