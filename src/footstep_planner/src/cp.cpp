#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "utilities/leg.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "legs_dynamics/robot_model.hpp"
#include "utilities/robot_state.hpp"
#include "footstep_planner/capture_point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int32.hpp"


#include <fstream>
#include <iomanip>
#include <algorithm>


using namespace std::chrono_literals;


// Footstep planner: from the current state, computes the capture point and the next
// footstep location (planar xy, relative to the stance foot).
//
// Deliberately owns no gait state. Swing trajectory generation and the stance switch
// live in the trajectory node; this node is a per-tick function of the robot state.
class CP_Node : public rclcpp::Node{
    public:
        CP_Node() : Node("cp_node") {
            std::string mjcf = this->declare_parameter<std::string>("mjcf_path", "");
            model_ = std::make_unique<dynamics::RobotModel>(mjcf);
            v_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/desired_vel", 10, std::bind(&CP_Node::vel_callback, this, std::placeholders::_1));
            js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&CP_Node::js_callback, this, std::placeholders::_1));
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/simulator/floating_base_state", 10, std::bind(&CP_Node::odom_callback, this, std::placeholders::_1));
            timer_ = this->create_wall_timer(25ms, std::bind(&CP_Node::capture_point, this));

            cp_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/capture_point", 10);
            step_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/step", 10);

            stance_sub_ = this->create_subscription<std_msgs::msg::Int32>("/stance", 10,
                 [this](const std_msgs::msg::Int32 &msg) {
                    stance_ = (msg.data == 0) ? legs::Side::Left : legs::Side::Right;
                    });


            cp_log_.open("cp_log.csv");
            cp_log_ << "time,xi_x,xi_y,step_x,step_y\n";
        }

        ~CP_Node() { cp_log_.close(); }


    private:
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr v_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr stance_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cp_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr step_pub_;
        

        Eigen::Vector3d x_dot_des_ {0.0, 0.0, 0.0};
        std::unique_ptr<dynamics::RobotModel> model_;
        robot::RobotState state_;
        capturepoint::CapturePoint cp_;

        std::ofstream cp_log_;

        bool map_built_ = false;
        bool base_ready_ = false;

        std::unordered_map<std::string, int> joint_map_;

        // TODO: source this from the trajectory node's gait state instead of owning it.
        legs::Side stance_ = legs::Side::Right;

        Eigen::Vector2d x_dot_prev_ = Eigen::Vector2d::Zero();
        Eigen::Vector2d x_ddot_filt_ = Eigen::Vector2d::Zero();
        double t_prev_a_ = 0.0;
        bool have_prev_a_ = false;

        const std::array<std::string, 6> joint_order_{
            "left_hip_yaw", "left_hip_pitch", "left_knee",
            "right_hip_yaw", "right_hip_pitch", "right_knee"
        };

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

        void vel_callback(const std_msgs::msg::Float64MultiArray &msg) {
            if (msg.data.size() != 3) {
                RCLCPP_WARN(this->get_logger(), "Desired velocity needs 3 values.");
                return;
            }
            x_dot_des_ = {msg.data[0], msg.data[1], msg.data[2]};
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

        void capture_point() {
            if (!map_built_ || !base_ready_) { return; }

            Eigen::Vector3d com = model_->comPosition();
            Eigen::Vector3d p_stance_w = model_->footPose(stance_).translation();
            // Position from the true CoM (removes the ~8 cm base-origin lateral bias);
            // velocity from the torso (clean, no swing-leg momentum spikes).
            Eigen::Vector2d x = (com - p_stance_w).head<2>();       // CoM position rel. stance foot
            Eigen::Vector2d x_dot = state_.base_lin_vel.head<2>();  // torso velocity
            // omega uses the CoM height ABOVE the stance foot, not absolute world z.
            // Guard: if the CoM drops to/below the foot (a fall), sqrt(g/h) -> NaN and
            // poisons the whole pipeline, so clamp the height to a positive minimum.
            cp_.set_height(std::max(com.z() - p_stance_w.z(), 0.15));

            double t = this->get_clock()->now().seconds();

            Eigen::Vector2d xi = cp_.compute_cp(x, x_dot);

            // Measured torso acceleration = low-pass-filtered finite-diff of x_dot
            // (matches the reference's ddx_com; NOT the analytic w^2*x, which cancels x).
            if (have_prev_a_ && (t - t_prev_a_) > 1e-6) {
                Eigen::Vector2d a_raw = (x_dot - x_dot_prev_) / (t - t_prev_a_);
                double alpha = 0.85;   // 0..1, higher = smoother / more lag
                x_ddot_filt_ = alpha * x_ddot_filt_ + (1.0 - alpha) * a_raw;
            }
            x_dot_prev_ = x_dot;
            t_prev_a_ = t;
            have_prev_a_ = true;

            Eigen::Vector2d step = cp_.compute_px(x, x_dot, x_ddot_filt_, x_dot_des_.head<2>());

            step.x() = std::clamp(step.x(), -0.2, 0.3);

            geometry_msgs::msg::Point step_msg;
            step_msg.x = step.x();
            step_msg.y = step.y();
            step_msg.z = 0.0;

            step_pub_->publish(step_msg);

            cp_log_
                << std::fixed << std::setprecision(6)
                << t << ","
                << xi.x() << "," << xi.y() << ","
                << step.x() << "," << step.y() << "\n";

            Eigen::Vector2d xi_world = xi + p_stance_w.head<2>();
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "odom";
            m.header.stamp = this->now();
            m.ns = "capture_point";
            m.id = 0;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = xi_world.x();
            m.pose.position.y = xi_world.y();
            m.pose.position.z = 0.0;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = 0.04;
            m.color.r = 1.0; m.color.a = 1.0;
            cp_pub_->publish(m);
        }

};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CP_Node>());
    rclcpp::shutdown();
    return 0;
}
