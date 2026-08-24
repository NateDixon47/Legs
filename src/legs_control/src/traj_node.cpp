#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "utilities/leg.hpp"
#include "utilities/robot_state.hpp"
#include "legs_dynamics/robot_model.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <fstream>
#include <iomanip>
#include <algorithm>
#include <cmath>


using namespace std::chrono_literals;


// Swing trajectory generator + gait state machine.
//
// Owns the gait state (which foot is stance, time within the step, when to switch)
// and turns the footstep planner's stance-relative foothold into a world-frame swing
// reference for the controller.
//
// Behaviour deliberately matches the original single-node implementation: xy tracks
// the live foothold directly, only z follows a trajectory. The trajectory shape is
// the next thing to change, once this reproduces the old behaviour.
class Traj_Node : public rclcpp::Node{
    public:
        Traj_Node() : Node("traj_node") {
            std::string mjcf = this->declare_parameter<std::string>("mjcf_path", "");
            model_ = std::make_unique<dynamics::RobotModel>(mjcf);

            step_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/step", 10,
                std::bind(&Traj_Node::step_callback, this, std::placeholders::_1));
            js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
                std::bind(&Traj_Node::js_callback, this, std::placeholders::_1));
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/simulator/floating_base_state", 10,
                std::bind(&Traj_Node::odom_callback, this, std::placeholders::_1));

            foot_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/foot_pos", 10);
            swing_vel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/swing_vel", 10);
            stance_pub_ = this->create_publisher<std_msgs::msg::Int32>("/stance", 10);

            // Runs at the controller's rate so the reference isn't a 25 ms staircase.
            timer_ = this->create_wall_timer(2ms, std::bind(&Traj_Node::traj_callback, this));

            traj_log_.open("traj_log.csv");
            traj_log_ << "time,stance,t_swing,tau,contact_l,contact_r,has_lifted,"
                      << "step_x,step_y,des_x,des_y,des_z,vz_des,act_x,act_y,act_z,switched\n";
        }

        ~Traj_Node() { traj_log_.close(); }

    private:
        // --- inputs ------------------------------------------------------------

        void step_callback(const geometry_msgs::msg::Point &msg) {
            // Planar foothold, origin at the stance foot, world-aligned axes.
            step_ = Eigen::Vector2d(msg.x, msg.y);
            have_step_ = true;
        }

        void js_callback(const sensor_msgs::msg::JointState &msg) {
            if (!map_built_) { build_map(msg); }
            if (!map_built_) { return; }

            for (size_t i = 0; i < joint_order_.size(); i++) {
                int k = joint_map_.at(joint_order_[i]);
                state_.q(i) = msg.position[k];
                if (k < static_cast<int>(msg.velocity.size())) {
                    state_.q_dot(i) = msg.velocity[k];
                }
            }

            model_->setState(state_);
        }

        void odom_callback(const nav_msgs::msg::Odometry &msg) {
            const auto &p = msg.pose.pose.position;
            const auto &o = msg.pose.pose.orientation;
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.translation() = Eigen::Vector3d(p.x, p.y, p.z);
            pose.linear() = Eigen::Quaterniond(o.w, o.x, o.y, o.z).toRotationMatrix();
            state_.base_pose = pose;

            state_.base_lin_vel = Eigen::Vector3d(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
            state_.base_ang_vel = Eigen::Vector3d(msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z);
            base_ready_ = true;
        }

        void build_map(const sensor_msgs::msg::JointState &msg) {
            joint_map_.clear();
            for (size_t i = 0; i < msg.name.size(); i++) {
                joint_map_[msg.name[i]] = i;
            }
            for (const auto &name : joint_order_) {
                if (joint_map_.find(name) == joint_map_.end()) {
                    RCLCPP_WARN(get_logger(), "waiting for joint '%s' in /joint_states", name.c_str());
                    return;
                }
            }
            map_built_ = true;
        }

        // --- gait state machine ------------------------------------------------

        // Liftoff latch: contact only counts as touchdown once the foot has actually
        // left the ground during this step. Without it the swing foot is still planted
        // at t_swing = 0 and the step would end the instant it began.
        bool update_gait() {
            const bool swing_contact = model_->inContact(swing_);
            if (!swing_contact) { has_lifted_ = true; }

            const bool touchdown = has_lifted_ && swing_contact;
            const bool timeout   = (t_swing_ >= T_max_);

            if (touchdown || timeout) {
                if (timeout && !touchdown) {
                    RCLCPP_WARN(get_logger(), "step timed out at %.3f s without touchdown", t_swing_);
                }
                switch_stance();
                return true;
            }
            return false;
        }

        void switch_stance() {
            std::swap(stance_, swing_);
            t_swing_ = 0.0;
            has_lifted_ = false;
        }

        // --- swing reference ---------------------------------------------------

        void traj_callback() {
            if (!map_built_ || !base_ready_) { return; }

            if (!initialized_) {
                ground_z_ = model_->footPose(stance_).translation().z();
                initialized_ = true;
            }

            // Gait first, so everything published below reflects the post-switch state.
            const bool switched = update_gait();

            // Stance is published unconditionally: the footstep planner needs it before
            // it can produce the /step this node is waiting on.
            std_msgs::msg::Int32 stance_msg;
            stance_msg.data = (stance_ == legs::Side::Left) ? 0 : 1;
            stance_pub_->publish(stance_msg);

            if (!have_step_) { return; }

            const Eigen::Vector3d p_stance_w = model_->footPose(stance_).translation();
            const Eigen::Vector3d p_swing_w  = model_->footPose(swing_).translation();

            // xy: track the foothold directly (no interpolation) -- matches the original.
            // z: sine arc, ground -> apex at mid-swing -> ground at touchdown.
            const double tau = std::clamp(t_swing_ / T_, 0.0, 1.0);
            Eigen::Vector3d p_des;
            p_des.head<2>() = step_ + p_stance_w.head<2>();
            p_des.z() = ground_z_ + apex_ * std::sin(M_PI * tau);

            // Past touchdown the arc's position is clamped, so its slope must be zero
            // too. Reporting the unclamped derivative commands a persistent downward
            // velocity for the whole overrun, driving the swing foot into the ground.
            const double vz_des = (tau >= 1.0) ? 0.0
                                               : apex_ * (M_PI / T_) * std::cos(M_PI * tau);

            // --- publish (left = [0:3], right = [3:6], world frame) ---
            // The stance slot carries the measured stance foot; the controller force-
            // controls that leg and ignores it, but publishing the measurement rather
            // than a remembered command keeps the message honest.
            const Eigen::Vector3d left  = (stance_ == legs::Side::Left) ? p_stance_w : p_des;
            const Eigen::Vector3d right = (stance_ == legs::Side::Left) ? p_des : p_stance_w;

            std_msgs::msg::Float64MultiArray foot_msg;
            foot_msg.data = {left.x(), left.y(), left.z(), right.x(), right.y(), right.z()};
            foot_pub_->publish(foot_msg);

            std_msgs::msg::Float64 vel_msg;
            vel_msg.data = vz_des;
            swing_vel_pub_->publish(vel_msg);

            const double t = this->get_clock()->now().seconds();
            traj_log_
                << std::fixed << std::setprecision(6)
                << t << ","
                << ((stance_ == legs::Side::Left) ? 0 : 1) << ","
                << t_swing_ << "," << tau << ","
                << model_->inContact(legs::Side::Left) << ","
                << model_->inContact(legs::Side::Right) << ","
                << has_lifted_ << ","
                << step_.x() << "," << step_.y() << ","
                << p_des.x() << "," << p_des.y() << "," << p_des.z() << ","
                << vz_des << ","
                << p_swing_w.x() << "," << p_swing_w.y() << "," << p_swing_w.z() << ","
                << (switched ? 1 : 0) << "\n";

            t_swing_ += dt_;
        }

        // --- members -----------------------------------------------------------

        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr step_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr foot_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr swing_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr stance_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        std::unique_ptr<dynamics::RobotModel> model_;
        robot::RobotState state_;

        std::ofstream traj_log_;

        Eigen::Vector2d step_ = Eigen::Vector2d::Zero();
        bool have_step_ = false;

        bool map_built_ = false;
        bool base_ready_ = false;
        bool initialized_ = false;
        std::unordered_map<std::string, int> joint_map_;

        // Gait state -- this node is the single owner.
        legs::Side stance_ = legs::Side::Right;
        legs::Side swing_  = legs::Side::Left;
        double t_swing_ = 0.0;
        bool has_lifted_ = false;

        const double dt_ = 0.002;   // must match the timer period
        double T_ = 0.25;           // nominal swing duration
        double T_max_ = 2.0 * T_;   // hard timeout so a step can never stall
        double apex_ = 0.05;        // swing height above ground
        double ground_z_ = 0.0;

        const std::array<std::string, 6> joint_order_{
            "left_hip_yaw", "left_hip_pitch", "left_knee",
            "right_hip_yaw", "right_hip_pitch", "right_knee"
        };
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Traj_Node>());
    rclcpp::shutdown();
    return 0;
}
