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

#include "legs_control/wbc.hpp"

#include <fstream>
#include <iomanip>


using namespace std::chrono_literals;

// QP whole-body controller -- Stage C.
//
// Structurally identical to inverse_dynamics_node, with one difference: the swing
// leg's joint torques come from the QP instead of a 3x3 Jacobian inverse.
//
// All six joint torques come from the QP. There is no separate stance controller:
// height and attitude are tasks on the base, the stance foot is held by the contact
// constraint, and gravity compensation is implicit in the bias forces h.
//
//   tasks       swing foot (3) + base z/roll/pitch (3) = 6, exactly the number of
//               free dimensions, so the tasks do not compete for weight.
//   untasked    base x, y and yaw -- horizontal balance is the footstep planner's
//               job via the capture point, not the stance ankle's.
//
// Gait state is owned by the trajectory node; this node only consumes it.
class wbc_controller_node : public rclcpp::Node{
    public:
        wbc_controller_node() : Node("wbc_node"), z_target_(0.475), stance_(legs::Side::Right){
            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);
            p_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/foot_pos", 10, std::bind(&wbc_controller_node::foot_pos_callback, this, std::placeholders::_1));
            js_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&wbc_controller_node::js_callback, this, std::placeholders::_1));
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/simulator/floating_base_state", 10, std::bind(&wbc_controller_node::odom_callback, this, std::placeholders::_1));
            stance_sub_ = this->create_subscription<std_msgs::msg::Int32>("/stance", 10,
                 [this](const std_msgs::msg::Int32 &msg) {
                    stance_ = (msg.data == 0) ? legs::Side::Left : legs::Side::Right;
                    log_stance_ = (stance_ == legs::Side::Left) ? 0 : 1;
                    });

            std::string mjcf = this->declare_parameter<std::string>("mjcf_path", "");
            model_ = std::make_unique<dynamics::RobotModel>(mjcf);

            // Must come after model_ -- the WBC needs nv to size its matrices.
            wbc_ = std::make_unique<control::WholeBodyController>(model_->nv(), wbc_reg_);
            w_base_ << 0.0, 0.0, w_base_z_,  w_base_rp_, w_base_rp_, 0.0;

            timer_ = this->create_wall_timer(2ms, std::bind(&wbc_controller_node::control_loop, this));

            // Distinct filenames so a WBC run does not overwrite an inverse_dynamics run.
            log_file_.open("wbc_torque_log.csv");
            q_log_file_.open("wbc_q_log.csv");
            height_log_file_.open("wbc_height_log.csv");
            rotation_log_file_.open("wbc_rot_log.csv");
            posture_err_log_file_.open("wbc_posture_e.csv");
            force_log_.open("wbc_force_log.csv");
            qp_log_.open("wbc_qp_log.csv");

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

            // QP diagnostics: solver health, the contact force it chose, and how well
            // the swing task was met. Watch solve_ok and swing_err first.
            qp_log_ << "time," << "solve_ok," << "fail_count,"
                    << "Fc_x," << "Fc_y," << "Fc_z,"
                    << "swing_err\n";
        }

        ~wbc_controller_node() {
            log_file_.close();
            q_log_file_.close();
            height_log_file_.close();
            rotation_log_file_.close();
            posture_err_log_file_.close();
            force_log_.close();
            qp_log_.close();
        }

    private:

        void foot_pos_callback(const std_msgs::msg::Float64MultiArray &msg) {
            if (msg.data.size() < 9) return;
            // Pick the SWING leg's world-frame target (opposite of stance).
            p_des_ = {msg.data[0], msg.data[1], msg.data[2]};
            v_des_ = {msg.data[3], msg.data[4], msg.data[5]};
            a_des_ = {msg.data[6], msg.data[7], msg.data[8]};
            have_p_des_ = true;
        }

        void control_loop() {
            if (!map_built_ || !base_ready_) return;

            double t = this->get_clock()->now().seconds();

            Eigen::VectorXd tau;

            Eigen::VectorXd q = state_.q;
            Eigen::VectorXd q_dot = state_.q_dot;

            const int swing0 = (stance_ == legs::Side::Left) ? 3 : 0;

            // ================================================================
            // BASE REFERENCE -- what the torso should do.
            // These used to be PD force/moment laws mapped through J^T. Now they are
            // just desired ACCELERATIONS handed to the QP, which derives the contact
            // force and every joint torque itself. Gravity compensation is not here
            // any more either -- it lives in the bias forces h.
            // ================================================================

            Eigen::Vector3d p_foot = model_->footPose(stance_).translation();
            Eigen::Vector3d p_base = state_.base_pose.translation();
            Eigen::Vector3d v_base = state_.base_lin_vel;

            // Vertical: hold torso height above the stance foot.
            double torso_z = p_base.z() - p_foot.z();
            // F.z() += Kp_h_ * (z_target_ - torso_z) - Kd_h_ * v_base.z();
            base_acc_(2) = Kp_z_ * (z_target_ - torso_z) - Kd_z_ * v_base.z();

            height_log_file_
                << std::fixed << std::setprecision(6)
                << t << ","
                << torso_z << ","
                << z_target_ << "\n";

            Eigen::Matrix3d R = state_.base_pose.rotation();
            Eigen::Matrix3d Rd = R_des_;

            // Vee map of the skew part = a small angle rotation-vector error [roll pitch yaw].
            // KEPT FOR LOGGING ONLY -- do not drive the task with this. Its magnitude is
            // sin(total rotation from Rd to R), yaw included, so an untasked yaw drift
            // crushes the roll/pitch components toward zero and the tilt correction
            // silently fades. That is what ended the 25 s run: yaw reached -0.98 rad and
            // roll authority fell to ~30% of nominal.
            Eigen::Matrix3d skew = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
            Eigen::Vector3d e_rot(skew(2,1), skew(0,2), skew(1,0));

            // Tilt error from GRAVITY instead. g_b is world "up" in body coordinates --
            // on hardware this is just the normalised accelerometer reading, so nothing
            // here needs a world-frame pose.
            //
            // e_tilt = ez x g_b = (-g_b.y, +g_b.x, 0): its z component is identically
            // zero, so heading cannot contaminate it at any yaw.
            const Eigen::Vector3d g_b    = R.transpose() * Eigen::Vector3d::UnitZ();
            const Eigen::Vector3d e_tilt = Eigen::Vector3d::UnitZ().cross(g_b);

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

            // Roll/pitch reference. Yaw is left free -- see w_base_.
            Eigen::Vector3d omega = state_.base_ang_vel;

            // +Kp, -Kd: e_tilt already points along the CORRECTION axis, not the error
            // axis, so the proportional term is positive while the damping opposes omega.
            // Both e_tilt and omega are body-frame, which is why Jb's angular rows are
            // set to identity below.
            base_acc_(3) = Kp_rp_ * e_tilt.x() - Kd_rp_ * omega.x();
            base_acc_(4) = Kp_rp_ * e_tilt.y() - Kd_rp_ * omega.y();
            base_acc_(5) = 0.0;   // yaw untasked; e_tilt.z is identically zero anyway

            // ================================================================
            // THE QP -- now the ONLY source of joint torques, for both legs.
            // ================================================================
            legs::Side swing_side = (stance_ == legs::Side::Left) ? legs::Side::Right : legs::Side::Left;

            // Full-width Jacobians (3 x nv), NOT the 3x3 leg blocks the old node used.
            // The QP reasons about all 12 DOFs, so it needs every column.
            Eigen::Vector3d p_foot_sw     = model_->footPose(swing_side).translation();
            Eigen::MatrixXd Jsw_full      = model_->footJacobian(swing_side);
            Eigen::MatrixXd Jsw_dot_full  = model_->footJacobianDot(swing_side);
            // baseJacobian()'s angular rows are R (body -> world). The tilt task is
            // expressed in the body frame, so replace them with the identity: MuJoCo
            // stores free-joint qvel[3:6] as BODY-frame angular velocity, so I is the
            // correct map there. Linear rows are [I|0|0] already and stay world-frame,
            // which is what the height task wants ("up" comes from the same IMU).
            //
            // Both blocks are then constant, so Jb_dot is exactly zero -- consistent
            // with baseJacobianDot()*v being identically zero anyway (w x w = 0).
            Eigen::MatrixXd Jb = model_->baseJacobian();
            Jb.block(3, 3, 3, 3).setIdentity();
            const Eigen::MatrixXd Jb_dot = Eigen::MatrixXd::Zero(6, model_->nv());

            // Foot velocity from LEG MOTION ONLY, matching inverse_dynamics_node exactly.
            // The true world velocity is Jsw_full * v, which also includes base motion --
            // switching to it changes the damping term, so leave it alone until the QP
            // itself is validated in the loop. One variable at a time.
            Eigen::Vector3d qd_sw     = q_dot.segment<3>(swing0);
            Eigen::Vector3d v_foot_sw = Jsw_full.block<3,3>(0, 6 + swing0) * qd_sw;

            // Hold the current foot position until the first target arrives (avoids a startup yank).
            Eigen::Vector3d p_des = have_p_des_ ? p_des_ : p_foot_sw;

            // The feedback law is unchanged. a_des is the commanded foot acceleration;
            // the WBC takes it as given and knows nothing about these gains.
            Eigen::Vector3d x_ddot = a_des_ + Kp_s.cwiseProduct(p_des - p_foot_sw)
                                   - Kd_s.cwiseProduct(v_foot_sw)
                                   + Kff_s.cwiseProduct(v_des_);

            // Generalised velocity, MuJoCo DOF order: 3 base linear, 3 base angular,
            // then the 6 joints. Every Jdot*v term in the QP uses this.
            Eigen::VectorXd v(model_->nv());
            v.segment<3>(0) = state_.base_lin_vel;
            v.segment<3>(3) = state_.base_ang_vel;
            v.tail<6>()     = q_dot;

            wbc_->reset();
            wbc_->setDynamics(model_->massMatrix(), model_->biasForces());
            wbc_->setStanceContact(model_->footJacobian(stance_), model_->footJacobianDot(stance_), v);
            wbc_->addSwingTask(Jsw_full, Jsw_dot_full, v, x_ddot, swing_weight_);
            wbc_->addStanceTask(Jb, Jb_dot, v, base_acc_, w_base_);

            const bool ok = (wbc_->solve() == control::WholeBodyController::Status::kOk);
            
            if (ok) {
                tau = wbc_->torques();     // all 6: stance and swing both come from the QP
                tau_prev_ = tau;
            } else {
                // Nothing else is driving the legs now, so a failed solve means coasting
                // on stale torques for BOTH legs. Watch fail_count in wbc_qp_log.csv.
                tau = tau_prev_;
                ++qp_fail_count_;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                    "WBC solve failed (%d total) -- holding last torques",
                                    qp_fail_count_);
            }

            double swing_err = ok
                ? (Jsw_full * wbc_->accelerations() + Jsw_dot_full * v - x_ddot).norm()
                : -1.0;
            const Eigen::Vector3d Fc = ok ? wbc_->contactForce() : Eigen::Vector3d::Zero();

            // Now the QP's contact force, not a force the node computed itself.
            force_log_
                << std::fixed << std::setprecision(6)
                << t << ","
                << Fc.x() << ","
                << Fc.y() << ","
                << Fc.z() << "\n";

            qp_log_ << std::fixed << std::setprecision(6)
                    << t << "," << (ok ? 1 : 0) << "," << qp_fail_count_ << ","
                    << Fc.x() << "," << Fc.y() << "," << Fc.z() << ","
                    << swing_err << "\n";
            // ----------------------------------------------------------------


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

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr p_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_subscriber_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr stance_sub_;

        std::unique_ptr<dynamics::RobotModel> model_;
        std::unique_ptr<control::WholeBodyController> wbc_;
        robot::RobotState state_;

        // Swing-foot task-space reference and gains.
        Eigen::Vector3d p_des_ = Eigen::Vector3d::Zero();
        bool have_p_des_ = false;
        Eigen::Vector3d v_des_ = Eigen::Vector3d::Zero();   // swing-foot velocity reference
        Eigen::Vector3d a_des_ = Eigen::Vector3d::Zero();
        // 6 long: 3 linear then 3 angular, matching baseJacobian's row order.
        Eigen::VectorXd base_acc_ = Eigen::VectorXd::Zero(6);

        Eigen::Vector3d Kp_s {1600.0, 1600.00, 1600.0};   // N/m
        Eigen::Vector3d Kd_s {80.0, 64.0, 64.0};         // N.s/m   damping on measured velocity
        Eigen::Vector3d Kff_s {80.0, 64.0, 100.0};        // N.s/m   feedforward on v_des

        // QP parameters.
        //
        // reg must stay >= 1e-4 while the swing task is the only one: it makes P
        // rank 15 instead of rank 3. Below that OSQP reports dual infeasible and
        // returns garbage. Lower it once Stage D adds CoM and orientation tasks.
        double wbc_reg_ {1e-4};

        // 2.0 because addSwingTask uses P += w*J'J rather than 2w*J'J, so the
        // effective weight is w/2. This value reproduces the Stage C reference
        // torques [-24.05, 4.80, -19.23 | -3.01, 1.86, 1.23].
        double swing_weight_ {2.0};

        // Base-task weights, one per axis of baseJacobian:
        //   [x, y, z | roll, pitch, yaw]
        // x, y and yaw are deliberately ZERO -- horizontal balance and heading are
        // the footstep planner's job, and tasking them fights the capture point.
        // That leaves 3 base dimensions + 3 swing = 6, exactly the free dimensions,
        // so the tasks do not compete and the magnitude barely matters above ~5.
        double w_base_z_  {10.0};
        double w_base_rp_ {10.0};
        Eigen::VectorXd w_base_ = Eigen::VectorXd::Zero(6);

        // Base-task gains in ACCELERATION units (1/s^2 and 1/s), NOT force units.
        // Kp_z_ = Kp_h_/mass = 2000/12.736, Kd_z_ = Kd_h_/mass = 300/12.736: the QP
        // derives the force itself from M, so mass must not be baked into the gain.
        // (300/12.736 lands at damping ratio 0.94 -- near critical, as tuned.)
        double Kp_z_  {157.0};
        double Kd_z_  {23.6};
        double Kp_rp_ {150.0};
        double Kd_rp_ {25.0};

        Eigen::VectorXd tau_prev_ = Eigen::VectorXd::Zero(6);
        int qp_fail_count_ = 0;

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
        std::ofstream qp_log_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wbc_controller_node>());
    rclcpp::shutdown();
    return 0;
}
