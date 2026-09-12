#ifndef LEGS_DYNAMICS_ROBOT_MODEL_HPP
#define LEGS_DYNAMICS_ROBOT_MODEL_HPP

#include <array>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <mujoco/mujoco.h>

#include "utilities/leg.hpp"          // legs::Side
#include "utilities/robot_state.hpp"  // robot::RobotState

namespace dynamics {

// Custom deleters so unique_ptr frees MuJoCo objects with the right functions.
struct MjModelDeleter {
    void operator()(mjModel* m) const { mj_deleteModel(m); }
};
struct MjDataDeleter {
    void operator()(mjData* d) const { mj_deleteData(d); }
};

// Wraps MuJoCo as a dynamics query engine for the bipedal robot. Owns its own
// model+data (separate from the simulator): you set a state, then read the mass
// matrix, bias/gravity forces, CoM, and foot poses/Jacobians at that state.
//
// All getters are valid only after a setState() call (which runs mj_forward).
class RobotModel {
 public:
    explicit RobotModel(const std::string& mjcf_path);

    // Non-copyable (owns raw MuJoCo resources), but movable (unique_ptr members).
    RobotModel(const RobotModel&) = delete;
    RobotModel& operator=(const RobotModel&) = delete;
    RobotModel(RobotModel&&) = default;
    RobotModel& operator=(RobotModel&&) = default;

    // Dimensions: nq=13 (3 pos + 4 quat + 6 joints), nv=12 (6 base + 6 joints), nu=6.
    int nq() const;
    int nv() const;
    int nu() const;

    // Set the evaluation point from raw MuJoCo-layout vectors (sizes nq and nv).
    void setState(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel);

    // Convenience: set a static configuration (zero velocity) from a base pose and
    // the 6 joint angles (ordered as joint_names_). This is the packing a future
    // RobotState overload would reuse.
    void setState(const Eigen::Isometry3d& base_pose,
                  const Eigen::VectorXd& joint_angles);

    // Set the full state (base pose + twist, joint positions + velocities) from a
    // RobotState snapshot. This is what the estimator/controllers will call.
    void setState(const robot::RobotState& state);

    Eigen::MatrixXd massMatrix() const;     // M(q)            nv x nv
    Eigen::VectorXd biasForces() const;     // C(q,q̇)q̇ + g(q)  nv
    Eigen::VectorXd gravityForces() const;  // g(q)            nv

    Eigen::Vector3d   comPosition() const;          // whole-body CoM, world frame
    Eigen::Isometry3d footPose(legs::Side) const;   // foot site pose, world frame
    Eigen::MatrixXd   footJacobian(legs::Side) const;  // dp_foot/dv     3 x nv
    Eigen::MatrixXd   footJacobianDot(legs::Side) const;
    Eigen::MatrixXd   baseJacobian() const;
    Eigen::MatrixXd   baseJacobianDot() const;

    Eigen::Vector3d comVelocity() const;

    Eigen::MatrixXd footJacobianFull(legs::Side side) const;

    // True when this foot is touching something (in practice, the floor).
    //
    // Reads the contacts mj_forward() already computed for the current state, so it
    // costs nothing extra and needs no <sensor> block in the MJCF. Uses contact
    // existence (a geometric test), not contact force — this model reconstructs state
    // from joint feedback without the true actuator state, so its constraint forces
    // are approximate while interpenetration is exact.
    //
    // SIM ONLY. On hardware this is replaced by a torque-residual estimator behind the
    // same accessor, so callers must not depend on where the signal comes from.
    bool inContact(legs::Side side) const;

 private:
    std::unique_ptr<mjModel, MjModelDeleter> model_;
    std::unique_ptr<mjData, MjDataDeleter> data_;

    int base_body_id_ = -1;
    int left_foot_site_id_ = -1;
    int right_foot_site_id_ = -1;
    int left_foot_body_id_ = -1;
    int right_foot_body_id_ = -1;

    // qpos/qvel addresses of the 6 actuated joints, in joint_names_ order.
    std::array<int, 6> joint_qposadr_{};
    std::array<int, 6> joint_dofadr_{};
};

}  // namespace dynamics

#endif  // LEGS_DYNAMICS_ROBOT_MODEL_HPP
