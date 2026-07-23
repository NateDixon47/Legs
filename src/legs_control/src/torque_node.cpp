#include <chrono>
#include <functional>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "utilities/leg.hpp"
#include "utilities/robot.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "legs_dynamics/robot_model.hpp"
#include "utilities/robot_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"

#include <fstream>
#include <iomanip>


using namespace std::chrono_literals;

class Torque_Node : public rclcpp::Node{
    public:
        Torque_Node() : Node("torque_node"), robot_(), Kp_(100.0), Kd_(5.0), stance_(legs::Side::Right), Kp_h_(300.0), Kd_h_(5.0), z_target_(0.475){
            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);
            p_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/foot_pos", 10, std::bind(&Torque_Node::IK_callback, this, std::placeholders::_1));
            js_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&Torque_Node::js_callback, this, std::placeholders::_1));
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/simulator/floating_base_state", 10, std::bind(&Torque_Node::odom_callback, this, std::placeholders::_1));
            stance_sub_ = this->create_subscription<std_msgs::msg::Int32>("/stance", 10,
                 [this](const std_msgs::msg::Int32 &msg) {
                    stance_ = (msg.data == 0) ? legs::Side::Left : legs::Side::Right;
                    });

            std::string mjcf = this->declare_parameter<std::string>("mjcf_path", "");
            model_ = std::make_unique<dynamics::RobotModel>(mjcf);

            timer_ = this->create_wall_timer(2ms, std::bind(&Torque_Node::PD_control, this));
            q_des_ << 0.125, 0.6, 1.15, -0.125, -0.6, 1.15;
            q_des_prev_ << 0.0, 0.6, 1.15, 0.0, -0.6, 1.15;

            log_file_.open("torque_log.csv");

            //header
            log_file_ << "time," << "left_hip_yaw," << "left_hip_pitch," << "left_knee,"
                      << "right_hip_yaw," << "right_hip_pitch," << "right_knee\n";
        }

        ~Torque_Node() { log_file_.close(); }

    private:
        robot::Robot robot_;

        void IK_callback(const std_msgs::msg::Float64MultiArray &msg) {
            if (msg.data.size() < 6){
                RCLCPP_WARN(this->get_logger(), "foot_pos needs 6 values: [left x,y,z, right x,y,z]");
                return;
            }
            Eigen::Vector3d p_left(msg.data[0], msg.data[1], msg.data[2]);
            Eigen::Vector3d p_right(msg.data[3], msg.data[4], msg.data[5]);

            auto q_des = robot_.generate_command(p_left, p_right);
            if (!q_des) {
                // RCLCPP_WARN(this->get_logger(), "IK: No solution for one or both foot targets");
                return;
            }

            // q_des_ = *q_des;

            last_target_ << p_left, p_right;     // remember what we commanded, for the FK check
            have_target_ = true;
        }

        void PD_control() {
            if (!map_built_ || !base_ready_) return;

            Eigen::VectorXd tau, tau_g;
            
            Eigen::VectorXd q = state_.q;
            Eigen::VectorXd q_dot_des;
            Eigen::VectorXd q_dot = state_.q_dot;
            
            // F = J_base
            Eigen::VectorXd g = model_->gravityForces();

            Eigen::MatrixXd Jt = model_->footJacobian(stance_).transpose();

            Eigen::Vector3d F = Jt.topRows<3>().completeOrthogonalDecomposition().solve(g.head<3>());

            // Height controller
            double foot_z = model_->footPose(stance_).translation().z();
            double torso_z = state_.base_pose.translation().z() - foot_z;
            double base_vel_z = state_.base_lin_vel.z();
            // F.z() += Kp_h_ * (z_target_ - torso_z) - Kd_h_ * base_vel_z;

            tau_g = g.bottomRows<6>() - Jt.bottomRows<6>() * F;
            // tau_g = g.tail<6>();

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "F=[%.1f, %.1f, %.1f] tau_g hip_y=%.1f hip_p=%.1f knee=%.1f", F.x(), F.y(), F.z(), tau_g(3), tau_g(4), tau_g(5));

            if (!have_q_des_prev_) {
                q_des_prev_ = q_des_;
                have_q_des_prev_ = true;
            }

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "q_des_=%.1f %.1f %.1f", q_des_(3), q_des_(4), q_des_(5));

            Eigen::Matrix3d R = state_.base_pose.rotation();
            Eigen::Matrix3d Rd = R_des_;

            // Vee map of the skew part = a small angle rotation-vector error [roll pitch yaw]
            Eigen::Matrix3d skew = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
            Eigen::Vector3d e_rot(skew(2,1), skew(0,2), skew(1,0));

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "e_rot=%.1f %.1f %.1f", e_rot(0), e_rot(1), e_rot(2));

            // PD moment on the torso, regulate the roll and pitch, leave the yaw alone
            Eigen::Vector3d omega = state_.base_ang_vel;
            Eigen::Vector3d M;
            M.x() = Kp_o_ * e_rot.x() - Kd_o_ * omega.x();
            M.y() = Kp_o_ * e_rot.y() - Kd_o_ * omega.y();
            M.z() = 0.0;

            // Map the stance joint torques via the rotational Jacobians joint columns
            Eigen::MatrixXd Jfull = model_->footJacobianFull(stance_);
            Eigen::MatrixXd Jr_joints = Jfull.bottomRows<3>().rightCols<6>();
            Eigen::VectorXd tau_orient = Jr_joints.transpose() * M;

            // tau_g += tau_orient;

            // q_dot_des = (q_des_ - q_des_prev_) / 0.002;
            q_des_prev_ = q_des_;
            // tau = Kp_ * (q_des_ - q) + Kd_ * (q_dot_des - q_dot) + tau_g;
            tau = Kp_ * (q_des_ - q) + Kd_ * (-q_dot) + tau_g;

            tau = tau.cwiseMax(-tau_max_).cwiseMin(tau_max_);

            std_msgs::msg::Float64MultiArray torque;
            torque.data.resize(6);
            for (Eigen::Index i = 0; i < tau.size(); ++i) {
                torque.data[static_cast<std::size_t>(i)] = tau(i);
            }
            torque.data[0] = 0.0;
            torque.data[1] = 0.0;
            torque.data[2] = 0.0;

            double t = this->get_clock()->now().seconds();

            log_file_
                << std::fixed << std::setprecision(6)
                << t << ","
                << torque.data[0] << ","
                << torque.data[1] << ","
                << torque.data[2] << ","
                << torque.data[3] << ","
                << torque.data[4] << ","
                << torque.data[5] << "\n";

            
            publisher_->publish(torque);
        }

        // Look up a joint's position by name (JointState ordering is not guaranteed).
        static bool get_joint(const sensor_msgs::msg::JointState &msg,
                              const std::string &name, double &out) {
            for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
                if (msg.name[i] == name) { out = msg.position[i]; return true; }
            }
            return false;
        }

        void js_callback(const sensor_msgs::msg::JointState &msg) {
            update_state(msg);
            fk_check(msg);
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

            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            //     "FK check: left err %.1f mm | right err %.1f mm", err_mm_l, err_mm_r);
        }

        void update_state(const sensor_msgs::msg::JointState &msg) {
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

        void odom_callback(const nav_msgs::msg::Odometry &msg) {
            const auto &p = msg.pose.pose.position;
            const auto &o = msg.pose.pose.orientation;
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.translation() = Eigen::Vector3d(p.x, p.y, p.z);
            pose.linear() = Eigen::Quaterniond(o.w, o.x, o.y, o.z).toRotationMatrix();
            state_.base_pose = pose;

            if (!r_des_set_) { R_des_ = pose.rotation(); r_des_set_ = true; }

            state_.base_lin_vel = Eigen::Vector3d(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
            state_.base_ang_vel = Eigen::Vector3d(msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z);
            base_ready_ = true;
        }

        Eigen::VectorXd last_target_ = Eigen::VectorXd::Zero(6);
        bool have_target_ = false;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr p_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_subscriber_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr stance_sub_;

        std::unique_ptr<dynamics::RobotModel> model_;
        robot::RobotState state_;

        Eigen::VectorXd q_des_ = Eigen::VectorXd::Zero(6);


        bool map_built_ = false;
        bool base_ready_ = false;
        std::unordered_map<std::string, int> joint_map_;

        const std::array<std::string, 6> joint_order_{
            "left_hip_yaw", "left_hip_pitch", "left_knee",
            "right_hip_yaw", "right_hip_pitch", "right_knee"
        };

        double Kp_;
        double Kd_;
        double tau_max_ {60.0};

        double Kp_h_;
        double Kd_h_;
        double z_target_;
        double Kp_o_ {100.0};
        double Kd_o_ {10.0};

        Eigen::Matrix3d R_des_;
        bool r_des_set_ = false;

        rclcpp::TimerBase::SharedPtr timer_;

        bool have_q_des_prev_ = false;
        Eigen::VectorXd q_des_prev_ = Eigen::VectorXd::Zero(6);

        legs::Side stance_;

        std::ofstream log_file_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Torque_Node>());
    rclcpp::shutdown();
    return 0;
}
