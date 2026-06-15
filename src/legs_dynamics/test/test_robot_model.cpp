#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "legs_dynamics/robot_model.hpp"
#include "utilities/leg.hpp"

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

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
