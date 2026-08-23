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
#include "std_msgs/msg/float64.hpp"

#include <fstream>
#include <iomanip>


using namespace std::chrono_literals;

class controller_node : public rclcpp::Node{
    public:
        controller_node() : Node("controller_node"), robot_(), z_target_(0.45), stance_(legs::Side::Right){
            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);
            p_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/foot_pos", 10, std::bind(&controller_node::foot_pos_callback, this, std::placeholders::_1));
            js_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&controller_node::js_callback, this, std::placeholders::_1));
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/simulator/floating_base_state", 10, std::bind(&controller_node::odom_callback, this, std::placeholders::_1));
            stance_sub_ = this->create_subscription<std_msgs::msg::Int32>("/stance", 10,
                 [this](const std_msgs::msg::Int32 &msg) {
                    stance_ = (msg.data == 0) ? legs::Side::Left : legs::Side::Right;
                    log_stance_ = (stance_ == legs::Side::Left) ? 0 : 1;
                    q_dot_des_.setZero();   // drop stale swing-velocity FF on stance switch
                    });

            swing_vel_sub_ = this->create_subscription<std_msgs::msg::Float64>("/swing_vel", 10, std::bind(&controller_node::swing_vel_callback, this, std::placeholders::_1));

            std::string mjcf = this->declare_parameter<std::string>("mjcf_path", "");
            model_ = std::make_unique<dynamics::RobotModel>(mjcf);

            timer_ = this->create_wall_timer(2ms, std::bind(&controller_node::stance_control, this));
            q_des_ << 0.125, 0.6, 1.15, 0.0, -0.7, 1.3;
            q_des_prev_ << 0.0, 0.6, 1.15, 0.0, -0.6, 1.15;

            log_file_.open("torque_log.csv");
            q_log_file_.open("q_log.csv");
            height_log_file_.open("height_log.csv");
            rotation_log_file_.open("rot_log.csv");
            posture_err_log_file_.open("posture_e.csv");
            force_log_.open("force_log.csv");

            //header
            log_file_ << "time," << "left_hip_yaw," << "left_hip_pitch," << "left_knee,"
                      << "right_hip_yaw," << "right_hip_pitch," << "right_knee\n";

            q_log_file_ << "time," << "q," << "stance," 
                        << "left_hip_yaw," << "left_hip_pitch," << "left_knee," << "right_hip_yaw," << "right_hip_pitch," << "right_knee," 
                        << "left_hip_yaw_d," << "left_hip_pitch_d," << "left_knee_d," << "right_hip_yaw_d," << "right_hip_pitch_d," << "right_knee_d\n";

            height_log_file_ << "time," << "torso_height," << "desired_height\n";
            rotation_log_file_ << "time," << "roll," << "pitch," << "yaw," << "roll_d," << "pitch_d," << "yaw_d\n";
            posture_err_log_file_ << "time," << "roll_e," << "pitch_e," << "yaw_e\n"; 
            force_log_ << "time," << "Fx," << "Fy," << "Fz\n";
        }

        ~controller_node() { log_file_.close(); q_log_file_.close(); height_log_file_.close(); rotation_log_file_.close(); posture_err_log_file_.close(); }

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
                RCLCPP_WARN(this->get_logger(), "IK: No solution for one or both foot targets");
                return;
            }

            // Velocity feedforward (Approach B): implied joint velocity from the change in
            // q_des over the /foot_pos interval (NOT the 2 ms loop dt), low-pass filtered.
            // double now = this->now().seconds();
            // double dt = now - t_last_fp_;
            // if (have_last_fp_ && dt > 1e-3) {
            //     Eigen::VectorXd qd = (*q_des - q_des_) / dt;         // q_des_ still holds the old target
            //     q_dot_des_ = alpha_fp_ * q_dot_des_ + (1.0 - alpha_fp_) * qd;
            // }
            // t_last_fp_ = now;
            // have_last_fp_ = true;

            q_des_ = *q_des;

            last_target_ << p_left, p_right;     // remember what we commanded, for the FK check
            have_target_ = true;
        }

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
            Eigen::VectorXd q_dot_des;
            Eigen::VectorXd q_dot = state_.q_dot;

            
            // F = J_base
            Eigen::VectorXd g = model_->gravityForces();

            Eigen::MatrixXd Jt = model_->footJacobian(stance_).transpose();

            Eigen::Vector3d F = Jt.topRows<3>().completeOrthogonalDecomposition().solve(g.head<3>());

            const int stance0 = (stance_ == legs::Side::Left) ? 0 : 3;
            const int swing0 = (stance_ == legs::Side::Left) ? 3 : 0;

            // --- stance-foot force feedback ---------------------------------
            // F is the ground-reaction force the stance foot applies to the body,
            // in the world frame (+z up), same convention as the height controller.
            // p_base is the CoM proxy (the CoM sits in the torso); we regulate it
            // relative to the stance foot.
            Eigen::Vector3d p_foot = model_->footPose(stance_).translation();
            Eigen::Vector3d p_base = state_.base_pose.translation();
            Eigen::Vector3d v_base = state_.base_lin_vel;

            // Vertical: hold torso height above the stance foot.
            double torso_z = p_base.z() - p_foot.z();
            F.z() += Kp_h_ * (z_target_ - torso_z) - Kd_h_ * v_base.z();

            // F.z() = std::clamp(F.z(), 0.0, 200.0);

            // Horizontal (ankle / CoP strategy): push the CoM back over the stance
            // foot to fight the inverted-pendulum divergence. The restoring GRF
            // opposes horizontal CoM offset (Kp_xy_) and CoM velocity (Kd_xy_).
            // This offloads the stepping controller for small disturbances; its
            // authority is bounded by the friction cone (clamped below) and
            // physically by the foot size (CoP can't leave the foot).
            // F.x() += -Kp_xy_ * (p_base.x() - p_foot.x()) - Kd_xy_ * v_base.x();
            // F.y() += -Kp_xy_ * (p_base.y() - p_foot.y()) - Kd_xy_ * v_base.y();

            // Keep the horizontal force inside the friction cone |F_xy| <= mu*F_z
            // (scale x,y together to preserve direction) so the foot doesn't slip.
            double f_xy_max = mu_ * std::max(F.z(), 0.0);
            double f_xy = std::hypot(F.x(), F.y());
            // if (f_xy > f_xy_max && f_xy > 1e-9) {
            //     double scale = f_xy_max / f_xy;
            //     F.x() *= scale;
            //     F.y() *= scale;
            // }

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
            // tau_g = g.tail<6>();


            if (!have_q_des_prev_) {
                q_des_prev_ = q_des_;
                have_q_des_prev_ = true;
            }

            // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 100, "q_des_=%.1f %.1f %.1f", q_des_(0), q_des_(1), q_des_(2));

            Eigen::Matrix3d R = state_.base_pose.rotation();
            Eigen::Matrix3d Rd = R_des_;

            // Vee map of the skew part = a small angle rotation-vector error [roll pitch yaw]
            Eigen::Matrix3d skew = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
            Eigen::Vector3d e_rot(skew(2,1), skew(0,2), skew(1,0));
            // Eigen::Vector3d e_rot = vee(skew);

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
            // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "e_rot=%.1f %.1f %.1f", e_rot(0), e_rot(1), e_rot(2));

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

            // q_dot_des = (q_des_ - q_des_prev_) / 0.002;
            q_des_prev_ = q_des_;
            // tau = Kp_ * (q_des_ - q) + Kd_ * (q_dot_des - q_dot) + tau_g;
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
            Eigen::Vector3d v_des(0.0, 0.0, swing_vel_z_);                                      // desired foot vel (world, z-only for now)

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
                << q[5] << ","
                << q_des_[0] << ","
                << q_des_[1] << ","
                << q_des_[2] << ","
                << q_des_[3] << ","
                << q_des_[4] << ","
                << q_des_[5] << "\n";

            // tau = Kp_ * (q_des_ - q) + Kd_ * (-q_dot) + tau_g;

            tau = tau.cwiseMax(-tau_max_).cwiseMin(tau_max_);

            std_msgs::msg::Float64MultiArray torque;
            torque.data.resize(6);
            for (Eigen::Index i = 0; i < tau.size(); ++i) {
                torque.data[static_cast<std::size_t>(i)] = tau(i);
            }
            // torque.data[0] = 0.0;
            // torque.data[1] = 0.0;
            // torque.data[2] = 0.0;

            // torque.data[5] = 0.0;


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

        void swing_vel_callback(const std_msgs::msg::Float64 &msg) {
            swing_vel_z_ = msg.data;
        }

        Eigen::VectorXd last_target_ = Eigen::VectorXd::Zero(6);
        bool have_target_ = false;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr p_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_subscriber_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr stance_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr swing_vel_sub_;

        std::unique_ptr<dynamics::RobotModel> model_;
        robot::RobotState state_;

        Eigen::VectorXd q_des_ = Eigen::VectorXd::Zero(6);
        Eigen::Vector3d p_des_ = Eigen::Vector3d::Zero();
        bool have_p_des_ = false;
        // double Kp_s {100.0};   // task-space swing-foot position gain [N/m]
        // double Kd_s {1.0};     // task-space swing-foot velocity gain [N.s/m]
        Eigen::Vector3d Kp_s {100.0, 50.0, 1000.0};
        Eigen::Vector3d Kd_s {1.0, 3.0, 5.0};


        // Velocity feedforward (Approach B): filtered finite-diff of q_des at the /foot_pos rate.
        Eigen::VectorXd q_dot_des_ = Eigen::VectorXd::Zero(6);
        double swing_vel_z_ = 0.0;

        double t_last_fp_ = 0.0;
        bool have_last_fp_ = false;
        double alpha_fp_ = 0.7;   // low-pass on the FF velocity (higher = smoother/more lag)

        bool map_built_ = false;
        bool base_ready_ = false;
        std::unordered_map<std::string, int> joint_map_;

        const std::array<std::string, 6> joint_order_{
            "left_hip_yaw", "left_hip_pitch", "left_knee",
            "right_hip_yaw", "right_hip_pitch", "right_knee"
        };

        double Kp_ {30.0};
        double Kd_ {1.0};

        double tau_max_ {60.0};

        double Kp_h_ {2000.0}; // 2000
        double Kd_h_ {300.0}; // 300
        double z_target_;

        // Horizontal stance-foot (ankle/CoP) gains — start gentle and tune up
        // while watching for foot slip/tip. mu_ = friction-cone limit.
        double Kp_xy_ {400.0};
        double Kd_xy_ {25.0};
        double mu_ {0.8};

        double Kp_o_ {400.0};
        double Kd_o_ {10.0};

        Eigen::Matrix3d R_des_;
        bool r_des_set_ = false;

        rclcpp::TimerBase::SharedPtr timer_;

        bool have_q_des_prev_ = false;
        Eigen::VectorXd q_des_prev_ = Eigen::VectorXd::Zero(6);

        legs::Side stance_;
        int log_stance_;

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
