#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "legs_dynamics/robot_model.hpp"
#include "utilities/leg.hpp"
#include "utilities/robot_state.hpp"  // robot::RobotState

namespace {

// TODO: pass this in rather than hardcoding once we wire the model path properly.
const char* kModelPath =
    "/home/nate/legs_ws/src/legs_description/mujoco/scene.xml";

// A nominal standing-ish state: base lifted, identity orientation, joints at zero.
dynamics::RobotModel makeModelInNominalState() {
    dynamics::RobotModel model(kModelPath);
    Eigen::Isometry3d base = Eigen::Isometry3d::Identity();
    base.translation() = Eigen::Vector3d(0.0, 0.0, 0.55);
    model.setState(base, Eigen::VectorXd::Zero(6));
    return model;
}

}  // namespace

TEST(RobotModel, LoadsWithExpectedDimensions) {
    dynamics::RobotModel model(kModelPath);
    EXPECT_EQ(model.nq(), 13);  // 3 pos + 4 quat + 6 joints
    EXPECT_EQ(model.nv(), 12);  // 6 base + 6 joints
    EXPECT_EQ(model.nu(), 6);
}

TEST(RobotModel, MassMatrixIsSymmetricPositiveDefinite) {
    auto model = makeModelInNominalState();
    Eigen::MatrixXd M = model.massMatrix();

    ASSERT_EQ(M.rows(), 12);
    ASSERT_EQ(M.cols(), 12);
    EXPECT_LT((M - M.transpose()).norm(), 1e-9) << "mass matrix not symmetric";

    Eigen::LLT<Eigen::MatrixXd> llt(M);
    EXPECT_EQ(llt.info(), Eigen::Success) << "mass matrix not positive-definite";
}

TEST(RobotModel, BiasGravitySizeAndStateRestore) {
    auto model = makeModelInNominalState();

    EXPECT_EQ(model.biasForces().size(), 12);

    Eigen::MatrixXd M_before = model.massMatrix();
    Eigen::VectorXd g = model.gravityForces();

    EXPECT_EQ(g.size(), 12);
    EXPECT_GT(g.norm(), 0.0) << "gravity should be nonzero";
    // At zero velocity, the full bias equals gravity.
    EXPECT_LT((model.biasForces() - g).norm(), 1e-9);
    // gravityForces() must leave the model in the original state.
    EXPECT_LT((model.massMatrix() - M_before).norm(), 1e-12)
        << "gravityForces() did not restore state";
}

TEST(RobotModel, FootPoseAndJacobianAreFinite) {
    auto model = makeModelInNominalState();

    Eigen::Isometry3d lf = model.footPose(legs::Side::Left);
    Eigen::Isometry3d rf = model.footPose(legs::Side::Right);
    EXPECT_TRUE(lf.translation().allFinite());
    EXPECT_TRUE(rf.translation().allFinite());
    // Feet should be below the lifted base.
    EXPECT_LT(lf.translation().z(), 0.55);

    Eigen::MatrixXd J = model.footJacobian(legs::Side::Left);
    EXPECT_EQ(J.rows(), 3);
    EXPECT_EQ(J.cols(), 12);
    EXPECT_TRUE(J.allFinite());
}

// Fixed rotation from the URDF/Leg base_link frame to MuJoCo's base body frame.
// The URDF->MJCF conversion reoriented the base by 120 deg about (1,1,1) (the torso
// geom quat 0.5,0.5,0.5,0.5), which cyclically permutes axes: (a,b,c)_leg -> (c,a,b)_mj.
Eigen::Matrix3d mjFromLegBaseRotation() {
    Eigen::Matrix3d R;
    R << 0, 0, 1,
         1, 0, 0,
         0, 1, 0;
    return R;
}

// Cross-check: the foot position MuJoCo reports (world) must equal the same foot
// placed by our own kinematics, base_pose * (R * Leg::FK(angles)). This proves the
// MuJoCo dynamics model is the SAME robot as the validated FK/IK.
void expectFootMatchesFK(const Eigen::Isometry3d& base, const Eigen::VectorXd& q) {
    dynamics::RobotModel model(kModelPath);
    model.setState(base, q);

    const Eigen::Matrix3d R = mjFromLegBaseRotation();
    legs::Leg left(legs::Side::Left);
    legs::Leg right(legs::Side::Right);

    Eigen::Vector3d exp_left = base * (R * left.FK(q[0], q[1], q[2]));
    Eigen::Vector3d exp_right = base * (R * right.FK(q[3], q[4], q[5]));

    Eigen::Vector3d mj_left = model.footPose(legs::Side::Left).translation();
    Eigen::Vector3d mj_right = model.footPose(legs::Side::Right).translation();

    EXPECT_LT((mj_left - exp_left).norm(), 1e-4)
        << "left foot: MuJoCo " << mj_left.transpose()
        << " vs FK " << exp_left.transpose();
    EXPECT_LT((mj_right - exp_right).norm(), 1e-4)
        << "right foot: MuJoCo " << mj_right.transpose()
        << " vs FK " << exp_right.transpose();
}

TEST(RobotModel, FootPoseMatchesLegFK_IdentityBase) {
    Eigen::Isometry3d base = Eigen::Isometry3d::Identity();
    Eigen::VectorXd q0(6); q0 << 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd q1(6); q1 << 0.0, 0.3, 0.6, 0.0, -0.3, 0.6;
    Eigen::VectorXd q2(6); q2 << -0.4, 0.8, 1.2, 0.4, -0.8, 1.2;
    expectFootMatchesFK(base, q0);
    expectFootMatchesFK(base, q1);
    expectFootMatchesFK(base, q2);
}

TEST(RobotModel, FootPoseMatchesLegFK_TransformedBase) {
    // Base translated and rotated, to exercise the full world-frame composition.
    Eigen::Isometry3d base = Eigen::Isometry3d::Identity();
    base.translation() = Eigen::Vector3d(0.1, -0.2, 0.55);
    base.linear() = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::VectorXd q(6); q << 0.0, 0.3, 0.6, 0.0, -0.3, 0.6;
    expectFootMatchesFK(base, q);
}

TEST(RobotModel, SetStateRobotStateMatchesPositionOverload) {
    dynamics::RobotModel model(kModelPath);

    // A non-trivial base pose + joint configuration, with zero velocities.
    Eigen::Isometry3d base = Eigen::Isometry3d::Identity();
    base.translation() = Eigen::Vector3d(0.1, -0.2, 0.55);
    base.linear() =
        Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::VectorXd q(6);
    q << 0.0, 0.3, 0.6, 0.0, -0.3, 0.6;

    // Reference: the already-validated (base_pose, joints) overload.
    model.setState(base, q);
    Eigen::MatrixXd M_ref = model.massMatrix();
    Eigen::Vector3d com_ref = model.comPosition();
    Eigen::Isometry3d lf_ref = model.footPose(legs::Side::Left);

    // Same configuration expressed as a RobotState (velocities default to zero).
    robot::RobotState state;
    state.base_pose = base;
    state.q = q;
    model.setState(state);

    // Mass matrix, CoM, and foot pose depend only on configuration (not velocity),
    // so the two paths must agree to machine precision.
    EXPECT_LT((model.massMatrix() - M_ref).norm(), 1e-12);
    EXPECT_LT((model.comPosition() - com_ref).norm(), 1e-12);
    EXPECT_LT((model.footPose(legs::Side::Left).matrix() - lf_ref.matrix()).norm(),
              1e-12);
}

TEST(RobotModel, SetStateRobotStateAppliesVelocity) {
    dynamics::RobotModel model(kModelPath);

    Eigen::Isometry3d base = Eigen::Isometry3d::Identity();
    base.translation() = Eigen::Vector3d(0.0, 0.0, 0.55);
    Eigen::VectorXd q(6);
    q << 0.0, 0.3, 0.6, 0.0, -0.3, 0.6;

    robot::RobotState state;
    state.base_pose = base;
    state.q = q;

    // Velocities still zero: the full bias should equal gravity alone.
    model.setState(state);
    Eigen::VectorXd bias_static = model.biasForces();
    Eigen::VectorXd gravity = model.gravityForces();
    EXPECT_LT((bias_static - gravity).norm(), 1e-9);

    // Add joint velocities; .finished() returns the completed vector from the
    // comma-initializer temporary (<< yields a builder object, not the vector).
    state.q_dot = (Eigen::VectorXd(6) << 0.5, -0.4, 0.3, 0.2, -0.1, 0.6).finished();
    model.setState(state);
    Eigen::VectorXd bias_moving = model.biasForces();

    // Velocity reached qvel, so Coriolis/centrifugal terms must change the bias...
    EXPECT_GT((bias_moving - bias_static).norm(), 1e-6);
    // ...but gravity (evaluated at zero velocity) is unaffected by it.
    EXPECT_LT((model.gravityForces() - gravity).norm(), 1e-9);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
