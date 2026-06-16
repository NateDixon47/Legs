#include "legs_dynamics/robot_model.hpp"

#include <algorithm>
#include <stdexcept>

namespace dynamics {

namespace {
// Actuated joint order — must match the MJCF joint names and robot::Robot::joint_names_.
const std::array<std::string, 6> kJointNames = {
    "left_hip_yaw", "left_hip_pitch", "left_knee",
    "right_hip_yaw", "right_hip_pitch", "right_knee"};
}  // namespace

RobotModel::RobotModel(const std::string& mjcf_path) {
    char err[1000] = {0};
    mjModel* m = mj_loadXML(mjcf_path.c_str(), nullptr, err, sizeof(err));
    if (!m) {
        throw std::runtime_error("Failed to load MJCF: " + std::string(err));
    }
    model_.reset(m);
    data_.reset(mj_makeData(model_.get()));
    if (!data_) {
        throw std::runtime_error("mj_makeData failed.");
    }

    // Base body is the root of the kinematic tree (its subtree CoM = whole-robot CoM).
    base_body_id_ = mj_name2id(model_.get(), mjOBJ_BODY, "base");
    if (base_body_id_ == -1) {
        throw std::runtime_error("body 'base' not found in MJCF");
    }

    // Foot sites (added to the MJCF for contact pose/Jacobian).
    left_foot_site_id_ = mj_name2id(model_.get(), mjOBJ_SITE, "left_foot");
    right_foot_site_id_ = mj_name2id(model_.get(), mjOBJ_SITE, "right_foot");

    // Cache each actuated joint's qpos/qvel address so packing never hardcodes indices.
    for (std::size_t i = 0; i < kJointNames.size(); ++i) {
        int jid = mj_name2id(model_.get(), mjOBJ_JOINT, kJointNames[i].c_str());
        if (jid == -1) {
            throw std::runtime_error("joint '" + kJointNames[i] + "' not found in MJCF");
        }
        joint_qposadr_[i] = model_->jnt_qposadr[jid];
        joint_dofadr_[i] = model_->jnt_dofadr[jid];
    }
}

int RobotModel::nq() const { return model_->nq; }
int RobotModel::nv() const { return model_->nv; }
int RobotModel::nu() const { return model_->nu; }

void RobotModel::setState(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel) {
    if (qpos.size() != model_->nq || qvel.size() != model_->nv) {
        throw std::runtime_error("setState: qpos/qvel size mismatch");
    }
    std::copy(qpos.data(), qpos.data() + model_->nq, data_->qpos);
    std::copy(qvel.data(), qvel.data() + model_->nv, data_->qvel);
    // Forward pipeline: populates qM, qfrc_bias, kinematics, site poses, subtree_com.
    mj_forward(model_.get(), data_.get());
}

void RobotModel::setState(const Eigen::Isometry3d& base_pose,
                          const Eigen::VectorXd& joint_angles) {
    if (joint_angles.size() != 6) {
        throw std::runtime_error("setState: expected 6 joint angles");
    }
    Eigen::VectorXd qpos = Eigen::VectorXd::Zero(model_->nq);
    Eigen::VectorXd qvel = Eigen::VectorXd::Zero(model_->nv);  // static: zero velocity

    // Base free joint: position then quaternion. MuJoCo stores the quaternion
    // w-first, while Eigen's coeffs() are x,y,z,w — so write each component explicitly.
    qpos.segment<3>(0) = base_pose.translation();
    Eigen::Quaterniond r(base_pose.rotation());
    qpos[3] = r.w();
    qpos[4] = r.x();
    qpos[5] = r.y();
    qpos[6] = r.z();

    // Actuated joints, written to their resolved qpos addresses.
    for (std::size_t i = 0; i < joint_qposadr_.size(); ++i) {
        qpos[joint_qposadr_[i]] = joint_angles[static_cast<int>(i)];
    }

    setState(qpos, qvel);
}

void RobotModel::setState(const robot::RobotState& state) {
    Eigen::VectorXd qpos = Eigen::VectorXd::Zero(model_->nq);
    Eigen::VectorXd qvel = Eigen::VectorXd::Zero(model_->nv);

    qpos.segment<3>(0) = state.base_pose.translation();
    Eigen::Quaterniond rot(state.base_pose.rotation());
    qpos[3] = rot.w();
    qpos[4] = rot.x();
    qpos[5] = rot.y();
    qpos[6] = rot.z();

    qvel.segment<3>(0) = state.base_lin_vel;
    qvel.segment<3>(3) = state.base_ang_vel;

    for (std::size_t i = 0; i < joint_qposadr_.size(); i++) {
        qpos[joint_qposadr_[i]] = state.q[static_cast<int>(i)];
        qvel[joint_dofadr_[i]] = state.q_dot[static_cast<int>(i)];
    }

    setState(qpos, qvel);
}



Eigen::MatrixXd RobotModel::massMatrix() const {
    const int nv = model_->nv;
    Eigen::MatrixXd M(nv, nv);
    mj_fullM(model_.get(), M.data(), data_->qM);
    return M;
}

Eigen::VectorXd RobotModel::biasForces() const {
    return Eigen::Map<const Eigen::VectorXd>(data_->qfrc_bias, model_->nv);
}

Eigen::VectorXd RobotModel::gravityForces() const {
    const int nv = model_->nv;
    // g(q) = bias forces at q̇ = 0. Save velocity, zero it, re-forward, read, restore.
    Eigen::VectorXd qvel_saved = Eigen::Map<const Eigen::VectorXd>(data_->qvel, nv);

    std::fill(data_->qvel, data_->qvel + nv, 0.0);
    mj_forward(model_.get(), data_.get());
    Eigen::VectorXd g = Eigen::Map<const Eigen::VectorXd>(data_->qfrc_bias, nv);

    std::copy(qvel_saved.data(), qvel_saved.data() + nv, data_->qvel);
    mj_forward(model_.get(), data_.get());
    return g;
}

Eigen::Vector3d RobotModel::comPosition() const {
    return Eigen::Map<const Eigen::Vector3d>(data_->subtree_com + 3 * base_body_id_);
}

Eigen::Isometry3d RobotModel::footPose(legs::Side side) const {
    int sid = (side == legs::Side::Left) ? left_foot_site_id_ : right_foot_site_id_;
    if (sid == -1) {
        throw std::runtime_error("foot site not found; add <site> tags to the MJCF");
    }
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Map<const Eigen::Vector3d>(data_->site_xpos + 3 * sid);
    // site_xmat is a row-major flattened 3x3 rotation.
    pose.linear() = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
        data_->site_xmat + 9 * sid);
    return pose;
}

Eigen::MatrixXd RobotModel::footJacobian(legs::Side side) const {
    int sid = (side == legs::Side::Left) ? left_foot_site_id_ : right_foot_site_id_;
    if (sid == -1) {
        throw std::runtime_error("foot site not found; add <site> tags to the MJCF");
    }
    const int nv = model_->nv;
    // mj_jacSite fills the translational Jacobian as a row-major 3 x nv block.
    Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor> J(3, nv);
    mj_jacSite(model_.get(), data_.get(), J.data(), nullptr, sid);
    return J;  // converts to column-major MatrixXd on return
}

}  // namespace dynamics
