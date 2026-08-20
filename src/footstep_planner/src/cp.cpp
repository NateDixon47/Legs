#include <chrono>
#include <functional>
#include <string>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "utilities/leg.hpp"
#include "utilities/robot.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "legs_dynamics/robot_model.hpp"
#include "utilities/robot_state.hpp"
#include "footstep_planner/capture_point.hpp"
#include "utilities/leg.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"

#include <fstream>
#include <iomanip>
#include <algorithm>



using namespace std::chrono_literals;


class CP_Node : public rclcpp::Node{
    public:
        CP_Node() : Node("cp_node") {
            std::string mjcf = this->declare_parameter<std::string>("mjcf_path", "");
            model_ = std::make_unique<dynamics::RobotModel>(mjcf);
            v_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/desired_vel", 10, std::bind(&CP_Node::vel_callback, this, std::placeholders::_1));
            pos_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/foot_pos", 10);
            js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&CP_Node::js_callback, this, std::placeholders::_1));
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/simulator/floating_base_state", 10, std::bind(&CP_Node::odom_callback, this, std::placeholders::_1));
            timer_ = this->create_wall_timer(25ms, std::bind(&CP_Node::capture_point, this));

            cp_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/capture_point", 10);
            lf_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/left_foot", 10);
            stance_pub_ = this->create_publisher<std_msgs::msg::Int32>("/stance", 10);
            swing_vel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/swing_vel", 10);

            cp_log_.open("cp_log.csv");
            cp_log_ << "time," << "x," << "y\n";

            diag_log_.open("cp_diag.csv");
            diag_log_ << "time,x_x,x_y,xdot_x,xdot_y,xi_x,xi_y,fd_x,fd_y\n";

            step_log_.open("step_log");
            step_log_ << "time,xi_x,xi_y,step_x,step_y,des_x,des_y,des_z,act_x,act_y,act_z,err_vert,switched\n";
        }

        ~CP_Node() {cp_log_.close(); diag_log_.close(); step_log_.close(); }


    private:
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr v_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pos_publisher;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cp_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lf_pub_;

        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr stance_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr swing_vel_pub_;
        
        Eigen::Vector3d x_dot_des_ {0.0, 0.0, 0.0};
        std::unique_ptr<dynamics::RobotModel> model_; 
        robot::RobotState state_;
        capturepoint::CapturePoint cp_;

        std::ofstream cp_log_;
        std::ofstream diag_log_;
        std::ofstream step_log_;

        Eigen::Vector3d com_prev_ = Eigen::Vector3d::Zero();
        double t_prev_ = 0.0;
        bool have_prev_ = false;

        bool map_built_ = false;
        bool base_ready_ = false;

        std::unordered_map<std::string, int> joint_map_;

        double T_ = 0.25;
        double T_max_ = 2.0 * T_;
        double reach_warn_ = 0.4;
        double reach_max_ = 0.5;
        double speedup_ = 2.0;
        double swing_frac_ = 0.8;
        double t_land_ = swing_frac_ * T_;

        double height_ = 0.475;

        double dy_rocking_ = 0.1;
        double b_lat_ = 0.0;   // lateral swing-foot offset (outward, alternating) for frontal-plane balance

        double ground_z_ = 0.0;
        Eigen::Vector3d last_foothold_;


        legs::Side stance_ = legs::Side::Right;
        legs::Side swing_ = legs::Side::Left;

        Eigen::Vector2d p_start_;
        Eigen::Vector3d stance_anchor_w_;   // planted stance foot world position
        double t_swing_ = 0.0;              // current time in swing
        bool initialized_ = false;          // first-step anchors set?

        Eigen::Vector2d x_dot_prev_ = Eigen::Vector2d::Zero();
        Eigen::Vector2d x_ddot_filt_ = Eigen::Vector2d::Zero();
        double t_prev_a_ = 0.0;
        bool have_prev_a_ = false;


        // MuJoCo base body frame -> URDF base_link (leg) frame.
        // From the FK cross-check: foot_world = base_pose * (R_ * Leg::FK).
        Eigen::Matrix3d R_ = (Eigen::Matrix3d() << 0, 0, 1,
                                                   1, 0, 0,
                                                   0, 1, 0).finished();

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

        // Convert a world-frame point into the leg/base_link frame that Leg::IK expects.
        // world -> MuJoCo base (base_pose^-1) -> leg frame (R_^T).
        Eigen::Vector3d world_to_leg(const Eigen::Vector3d &p_world) const {
            return R_.transpose() * (state_.base_pose.inverse() * p_world);
        }

        void capture_point() {
            if (!map_built_ || !base_ready_) { return; }
            Eigen::Vector3d com = model_->comPosition();
            Eigen::Vector3d com_v = model_->comVelocity();
            Eigen::Vector3d p_stance_w = model_->footPose(stance_).translation();
            // Position from the true CoM (removes the ~8 cm base-origin lateral bias);
            // velocity from the torso (clean, no swing-leg momentum spikes).
            Eigen::Vector2d x = (com - p_stance_w).head<2>();       // CoM position rel. stance foot
            Eigen::Vector2d x_dot = state_.base_lin_vel.head<2>();  // torso velocity
            // omega uses the CoM height ABOVE the stance foot, not absolute world z.
            // Guard: if the CoM drops to/below the foot (a fall), sqrt(g/h) -> NaN and
            // poisons the whole pipeline, so clamp the height to a positive minimum.
            cp_.set_height(std::max(com.z() - p_stance_w.z(), 0.15));
            // --- CHECKPOINT: verify the stance-frame state reads correctly ---
            
            // RCLCPP_INFO(get_logger(), "Height: %.3f", com.z()-p_stance_w.z());
            // first-step init: anchor the current stance foot + swing liftoff before any transition
            if (!initialized_) {
                stance_anchor_w_ = p_stance_w;
                Eigen::Vector3d lifted_w = model_->footPose(swing_).translation();
                p_start_ = (lifted_w - p_stance_w).head<2>();
                ground_z_ = p_stance_w.z();
                initialized_ = true;
            }

            // --- foothold ---
            // Eigen::Vector2d xi     = cp_.compute_cp(x, x_dot);
            // Eigen::Vector2d xi_eos = cp_.predict_eos(xi, Eigen::Vector2d::Zero(), T_, t_swing_);

            // // --- capture-point diagnostics: decompose xi and validate comVelocity() ---
            double t_now = this->get_clock()->now().seconds();


            Eigen::Vector2d xi = cp_.compute_cp(x, x_dot);
            double w = cp_.get_omega();

            // Measured torso acceleration = low-pass-filtered finite-diff of x_dot
            // (matches the reference's ddx_com; NOT the analytic w^2*x, which cancels x).
            if (have_prev_a_ && (t_now - t_prev_a_) > 1e-6) {
                Eigen::Vector2d a_raw = (x_dot - x_dot_prev_) / (t_now - t_prev_a_);
                double alpha = 0.85;   // 0..1, higher = smoother / more lag
                x_ddot_filt_ = alpha * x_ddot_filt_ + (1.0 - alpha) * a_raw;
            }
            x_dot_prev_ = x_dot;
            t_prev_a_ = t_now;
            have_prev_a_ = true;

            Eigen::Vector2d x_dot_des = x_dot_des_.head<2>();
            // x_dot_des.y() += (stance_ == legs::Side::Right) ? -dy_rocking_ : dy_rocking_;

            Eigen::Vector2d step = cp_.compute_px(x, x_dot, x_ddot_filt_, x_dot_des);

            // Lateral foot-placement offset: place the swing foot a stance half-width
            // OUTSIDE the CoM, alternating by swing side, to create the frontal-plane
            // limit cycle. compute_px alone steps at the lateral CoM -> neutral/drift;
            // the outward offset is what bounces the CoM back. +y = left, so the
            // left swing foot gets +b_lat_, the right swing foot gets -b_lat_.
            double side = (swing_ == legs::Side::Left) ? 1.0 : -1.0;
            // step.y() += side * b_lat_;

            // if (stance_ == legs::Side::Right) step.y() = std::clamp(step.y(), 0.15, 0.3);
            // else step.y() = std::clamp(step.y(), -0.3, -0.15);

            step.x() = std::clamp(step.x(), -0.2, 0.3);

            // --- swing foot target: trajectory (stance frame) -> world -> leg frame ---
            // Eigen::Vector3d swing_pos  = cp_.swing_trajectory(T_, t_swing_, p_start_, step, 0.1);

            Eigen::Vector3d swing_pos;
            swing_pos.head<2>() = step;
            // swing_pos.z() = cp_.swing_height(t_swing_, T_, 0.05);
            Eigen::Vector2d swing_traj = cp_.swing_height(t_swing_, T_, 0.05);
            swing_pos.z() = swing_traj[0];
            
            // Publish swing foot velocity
            std_msgs::msg::Float64 swing_vel_msg;
            swing_vel_msg.data = swing_traj[1];
            swing_vel_pub_->publish(swing_vel_msg);

            Eigen::Vector3d swing_world;
            swing_world.head<2>() = swing_pos.head<2>() + p_stance_w.head<2>();
            swing_world.z() = ground_z_ + swing_pos.z();
            Eigen::Vector3d swing_leg  = world_to_leg(swing_pos + p_stance_w);

            // Swing target - pin z to the fixed ground + arc, not the measured stance z
            last_foothold_.head<2>() = step + p_stance_w.head<2>();
            last_foothold_.z() = ground_z_;

            // --- stance foot: held at its fixed world anchor ---
            Eigen::Vector3d stance_leg = world_to_leg(stance_anchor_w_);

            // --- pack by side (left = [0:3], right = [3:6]) and publish ---
            Eigen::Vector3d left  = (stance_ == legs::Side::Left) ? stance_leg : swing_leg;
            Eigen::Vector3d right = (stance_ == legs::Side::Left) ? swing_leg  : stance_leg;
            std_msgs::msg::Float64MultiArray msg;
            msg.data = {left.x(), left.y(), left.z(), right.x(), right.y(), right.z()};
            pos_publisher->publish(msg);


            // World footstep info 
            // swing_world = swing_pos + p_stance_w;
            Eigen::Vector3d stance_world = stance_anchor_w_;
            Eigen::Vector3d left_world = (stance_ == legs::Side::Left) ? stance_world : swing_world;

            visualization_msgs::msg::Marker lf_msg;
            lf_msg.header.frame_id = "odom";
            lf_msg.header.stamp = this->now();
            lf_msg.ns = "left_foot";
            lf_msg.id = 0;
            lf_msg.type = visualization_msgs::msg::Marker::SPHERE;
            lf_msg.action = visualization_msgs::msg::Marker::ADD;
            lf_msg.pose.position.x = left_world.x();
            lf_msg.pose.position.y = left_world.y();
            lf_msg.pose.position.z = left_world.z();
            lf_msg.pose.orientation.w = 1.0;
            lf_msg.scale.x = lf_msg.scale.y = lf_msg.scale.z = 0.04;
            lf_msg.color.b = 1.0; lf_msg.color.a = 1.0;
            lf_pub_->publish(lf_msg);

            // publish current stance
            std_msgs::msg::Int32 stance_msg;
            stance_msg.data = (stance_ == legs::Side::Left) ? 0 : 1; // 0 for left, 1 for right
            stance_pub_->publish(stance_msg);

            double t = this->get_clock()->now().seconds();

            cp_log_
                << std::fixed << std::setprecision(6)
                << t << ","
                << xi.x() << ","
                << xi.y() << "\n";

            visualization_msgs::msg::Marker m;

            Eigen::Vector2d xi_world = xi + p_stance_w.head<2>();
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

            Eigen::Vector3d swing_actual = model_->footPose(swing_).translation();
            Eigen::Vector3d foothold_world;
            foothold_world.head<2>() = step + p_stance_w.head<2>();
            foothold_world.z() = p_stance_w.z();
            double err_planar = (swing_actual.head<2>() - foothold_world.head<2>()).norm();
            double err_vert = (swing_actual.z() - foothold_world.z());
            
            // --- advance the step clock; transition when the step completes ---
            bool reached  = (t_swing_ >= T_ && err_planar < 0.075 && err_vert < 0.005);
            bool max_time = (t_swing_ >= T_max_);
            bool switched = reached || max_time;

            step_log_ << std::fixed << std::setprecision(6)
                << t << ","
                << xi.x()           << "," << xi.y()           << ","
                << step.x()         << "," << step.y()         << ","
                << swing_world.x()  << "," << swing_world.y()  << "," << swing_world.z()  << ","   // des = live commanded target
                << swing_actual.x() << "," << swing_actual.y() << "," << swing_actual.z() << ","
                << err_vert         << ","
                << (switched ? 1 : 0) << "\n";

            if (reached) {
                RCLCPP_INFO(get_logger(), "Reached pos");
                switch_stance();
            }
            // else if (max_time) {
            //     RCLCPP_INFO(get_logger(), "Max Time");
            //     switch_stance();
            // }

            t_swing_ += 0.025;
        }

        void switch_stance() {
            Eigen::Vector3d old_stance = stance_anchor_w_;
            std::swap(swing_, stance_);
            stance_anchor_w_ = last_foothold_;         // new pivot's landing spot
            // Eigen::Vector3d lifted_w = model_->footPose(swing_).translation();   // new swing's liftoff
            p_start_ = (old_stance - stance_anchor_w_).head<2>();
            cp_.switch_stance();
            t_swing_ = 0.0;
            // std_msgs::msg::Int32 msg;
            // msg.data = (stance_ == legs::Side::Left) ? 0 : 1; // 0 for left, 1 for right
            // stance_pub_->publish(msg);
        }

};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CP_Node>());
    rclcpp::shutdown();
    return 0;
}