#include <chrono>
#include <functional>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "utilities/leg.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "legs_dynamics/robot_model.hpp"
#include "utilities/robot_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"

#include <fstream>
#include <iomanip>


using namespace std::chrono_literals;

// Whole-body torque controller.
//
//   stance leg - gravity compensation + torso height PD + torso orientation PD,
//                mapped to joint torques through the stance foot Jacobian.
//   swing leg  - task-space (Cartesian) impedance about the reference published by
//                the trajectory node, mapped with J^T.
//
// Gait state is owned by the trajectory node; this node only consumes it.
class controller_node : public rclcpp::Node{
    public:
        controller_node() : Node("controller_node"), z_target_(0.45), stance_(legs::Side::Right){
            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);
            p_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/foot_pos", 10, std::bind(&controller_node::foot_pos_callback, this, std::placeholders::_1));
            js_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&controller_node::js_callback, this, std::placeholders::_1));
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/simulator/floating_base_state", 10, std::bind(&controller_node::odom_callback, this, std::placeholders::_1));
            stance_sub_ = this->create_subscription<std_msgs::msg::Int32>("/stance", 10,
                 [this](const std_msgs::msg::Int32 &msg) {
                    stance_ = (msg.data == 0) ? legs::Side::Left : legs::Side::Right;
                    log_stance_ = (stance_ == legs::Side::Left) ? 0 : 1;
                    });

            swing_vel_sub_ = this->create_subscription<std_msgs::msg::Float64>("/swing_vel", 10, std::bind(&controller_node::swing_vel_callback, this, std::placeholders::_1));

            std::string mjcf = this->declare_parameter<std::string>("mjcf_path", "");
            model_ = std::make_unique<dynamics::RobotModel>(mjcf);

            timer_ = this->create_wall_timer(2ms, std::bind(&controller_node::stance_control, this));

            log_file_.open("torque_log.csv");
            q_log_file_.open("q_log.csv");
            height_log_file_.open("height_log.csv");
            rotation_log_file_.open("rot_log.csv");
            posture_err_log_file_.open("posture_e.csv");
            force_log_.open("force_log.csv");

            //header
            log_file_ << "time," << "left_hip_yaw," << "left_hip_pitch," << "left_knee,"
                      << "right_hip_yaw," << "right_hip_pitch," << "right_knee\n";

            q_log_file_ << "time," << "stance,"
                        << "left_hip_yaw," << "left_hip_pitch," << "left_knee,"
                        << "right_hip_yaw," << "right_hip_pitch," << "right_knee\n";

            height_log_file_ << "time," << "torso_height," << "desired_height\n";
            rotation_log_file_ << "time," << "roll," << "pitch," << "yaw," << "roll_d," << "pitch_d," << "yaw_d\n";
            posture_err_log_file_ << "time," << "roll_e," << "pitch_e," << "yaw_e\n";
            force_log_ << "time," << "Fx," << "Fy," << "Fz\n";
        }

        ~controller_node() {
            log_file_.close();
            q_log_file_.close();
            height_log_file_.close();
            rotation_log_file_.close();
            posture_err_log_file_.close();
            force_log_.close();
        }

    private:

        void foot_pos_callback(const std_msgs::msg::Float64MultiArray &msg) {
            if (msg.data.size() < 6) return;
            // Pick the SWING leg's world-frame target (opposite of stance).
            if (stance_ == legs::Side::Left) { p_des_ = Eigen::Vector3d{msg.data[3], msg.data[4], msg.data[5]}; }
            else { p_des_ = Eigen::Vector3d{msg.data[0], msg.data[1], msg.data[2]}; }
            have_p_des_ = true;
        }

        void stance_control() {
            if (!map_built_ || !base_ready_) return;

            double t = this->get_clock()->now().seconds();

            Eigen::VectorXd tau, tau_g;

            Eigen::VectorXd q = state_.q;
            Eigen::VectorXd q_dot = state_.q_dot;

            Eigen::VectorXd g = model_->gravityForces();

            Eigen::MatrixXd Jt = model_->footJacobian(stance_).transpose();

            Eigen::Vector3d F = Jt.topRows<3>().completeOrthogonalDecomposition().solve(g.head<3>());

            const int swing0 = (stance_ == legs::Side::Left) ? 3 : 0;

            // --- stance-foot force feedback ---------------------------------
            // F is the ground-reaction force the stance foot applies to the body,
            // in the world frame (+z up), same convention as the height controller.
            // p_base is the CoM proxy (the CoM sits in the torso); we regulate it
            // relative to the stance foot.
            //
            // NOTE: the horizontal (ankle / CoP) strategy and its friction-cone clamp
            // were tried and removed -- stance-foot xy forces were destabilising. See
            // commit 9b6be28 to restore them.
            Eigen::Vector3d p_foot = model_->footPose(stance_).translation();
            Eigen::Vector3d p_base = state_.base_pose.translation();
            Eigen::Vector3d v_base = state_.base_lin_vel;

            // Vertical: hold torso height above the stance foot.
            double torso_z = p_base.z() - p_foot.z();
            F.z() += Kp_h_ * (z_target_ - torso_z) - Kd_h_ * v_base.z();

            force_log_
                << std::fixed << std::setprecision(6)
                << t << ","
                << F.x() << ","
                << F.y() << ","
                << F.z() << "\n";

            height_log_file_
                << std::fixed << std::setprecision(6)
                << t << ","
                << torso_z << ","
                << z_target_ << "\n";

            tau_g = g.bottomRows<6>() - Jt.bottomRows<6>() * F;

            Eigen::Matrix3d R = state_.base_pose.rotation();
            Eigen::Matrix3d Rd = R_des_;

            // Vee map of the skew part = a small angle rotation-vector error [roll pitch yaw]
            Eigen::Matrix3d skew = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
            Eigen::Vector3d e_rot(skew(2,1), skew(0,2), skew(1,0));

            Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2);
            Eigen::Vector3d rpy_d = Rd.eulerAngles(0, 1, 2);

            posture_err_log_file_
                << std::fixed << std::setprecision(6)
                << t << ","
                << e_rot.x() << ","
                << e_rot.y() << ","
                << e_rot.z() << "\n";

            rotation_log_file_
                << std::fixed << std::setprecision(6)
                << t << ","
                << rpy.x() << ","
                << rpy.y() << ","
                << rpy.z() << ","
                << rpy_d.x() << ","
                << rpy_d.y() << ","
                << rpy_d.z()<< "\n";

            // PD moment on the torso, regulate the roll and pitch, leave the yaw alone
            Eigen::Vector3d omega = state_.base_ang_vel;
            Eigen::Vector3d M;
            M.x() = Kp_o_ * e_rot.x() + Kd_o_ * omega.x();
            M.y() = Kp_o_ * e_rot.y() + Kd_o_ * omega.y();
            M.z() = 0.0;

            // Map the stance joint torques via the rotational Jacobians joint columns
            Eigen::MatrixXd Jfull = model_->footJacobianFull(stance_);
            Eigen::MatrixXd Jr_joints = Jfull.bottomRows<3>().rightCols<6>();
            Eigen::VectorXd tau_orient = Jr_joints.transpose() * M;

            tau_g += tau_orient;

            tau = tau_g;

            // --- task-space (Cartesian) swing-foot control ---------------------
            // Regulate the swing foot's WORLD position/velocity directly, then map the
            // Cartesian force to swing-joint torques with J^T (no IK, no J^-1, no leak).
            legs::Side swing_side = (stance_ == legs::Side::Left) ? legs::Side::Right : legs::Side::Left;

            Eigen::Vector3d p_foot_sw = model_->footPose(swing_side).translation();            // actual foot pos (world)
            Eigen::Matrix3d Jsw = model_->footJacobian(swing_side).block<3,3>(0, 6 + swing0);   // world, swing joints
            Eigen::Vector3d qd_sw = q_dot.segment<3>(swing0);                                   // swing joint velocities
            Eigen::Vector3d v_foot_sw = Jsw * qd_sw;                                            // actual foot vel (world)

            // Hold the current foot position until the first target arrives (avoids a startup yank).
            Eigen::Vector3d p_des = have_p_des_ ? p_des_ : p_foot_sw;
            Eigen::Vector3d v_des(0.0, 0.0, 0.0);   //swing_vel_z_  // desired foot vel (world, z-only for now)

            Eigen::Vector3d F_foot = Kp_s.cwiseProduct(p_des - p_foot_sw) + Kd_s.cwiseProduct(v_des - v_foot_sw);
            tau.segment<3>(swing0) = tau_g.segment<3>(swing0) + Jsw.transpose() * F_foot;       // keep swing gravity comp

            q_log_file_
                << std::fixed << std::setprecision(6)
                << t << ","
                << log_stance_ << ","
                << q[0] << ","
                << q[1] << ","
                << q[2] << ","
                << q[3] << ","
                << q[4] << ","
                << q[5] << "\n";

            tau = tau.cwiseMax(-tau_max_).cwiseMin(tau_max_);

            std_msgs::msg::Float64MultiArray torque;
            torque.data.resize(6);
            for (Eigen::Index i = 0; i < tau.size(); ++i) {
                torque.data[static_cast<std::size_t>(i)] = tau(i);
            }

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

        void js_callback(const sensor_msgs::msg::JointState &msg) {
            update_state(msg);
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

        void swing_vel_callback(const std_msgs::msg::Float64 &msg) {
            swing_vel_z_ = msg.data;
        }

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr p_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_subscriber_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr stance_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr swing_vel_sub_;

        std::unique_ptr<dynamics::RobotModel> model_;
        robot::RobotState state_;

        // Swing-foot task-space reference and gains.
        Eigen::Vector3d p_des_ = Eigen::Vector3d::Zero();
        bool have_p_des_ = false;
        double swing_vel_z_ = 0.0;
        Eigen::Vector3d Kp_s {100.0, 50.0, 1000.0};   // N/m
        Eigen::Vector3d Kd_s {1.0, 3.0, 5.0};         // N.s/m

        bool map_built_ = false;
        bool base_ready_ = false;
        std::unordered_map<std::string, int> joint_map_;

        const std::array<std::string, 6> joint_order_{
            "left_hip_yaw", "left_hip_pitch", "left_knee",
            "right_hip_yaw", "right_hip_pitch", "right_knee"
        };

        double tau_max_ {60.0};

        double Kp_h_ {2000.0};
        double Kd_h_ {300.0};
        double z_target_;

        double Kp_o_ {400.0};
        double Kd_o_ {10.0};

        Eigen::Matrix3d R_des_;
        bool r_des_set_ = false;

        rclcpp::TimerBase::SharedPtr timer_;

        legs::Side stance_;
        int log_stance_ = 1;

        std::ofstream log_file_;
        std::ofstream q_log_file_;
        std::ofstream rotation_log_file_;
        std::ofstream height_log_file_;
        std::ofstream posture_err_log_file_;
        std::ofstream force_log_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<controller_node>());
    rclcpp::shutdown();
    return 0;
}
