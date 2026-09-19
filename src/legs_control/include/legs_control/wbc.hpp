#pragma once

#include <Eigen/Dense>
#include "osqp.h"   // has its own extern "C" guards; osqp::osqp puts this dir on the path

namespace control {

class WholeBodyController {
 public:
    // reg regularises the cost. It must be large enough that P is well conditioned:
    // with only a swing task, P has rank 3 of nv+3, and OSQP reports "dual infeasible"
    // (returning garbage, not an error) below about 1e-4. Raise tasks -> lower reg.
    WholeBodyController(int nv, double reg);
    ~WholeBodyController();

    // Owns raw OSQP resources; copying would double-free.
    WholeBodyController(const WholeBodyController&) = delete;
    WholeBodyController& operator=(const WholeBodyController&) = delete;

    enum class Status { kOk, kSolverFailed, kNotSolved };

    // --- per tick, in this order -------------------------------------------
    void reset();                                                   // zero the accumulators
    void setDynamics(const Eigen::MatrixXd& M, const Eigen::VectorXd& h);
    void setStanceContact(const Eigen::MatrixXd& Jc, const Eigen::MatrixXd& Jc_dot, const Eigen::VectorXd& v);
    void addSwingTask(const Eigen::MatrixXd& Js, const Eigen::MatrixXd& Js_dot, const Eigen::VectorXd& v, const Eigen::Vector3d& a_des, double weight);
    Status solve();                                                 // returns, so it cannot be ignored

    // --- results, valid only when status() == kOk ---------------------------
    Status status() const { return status_; }
    const Eigen::VectorXd& torques() const { return tau_; }         // nv-6 actuated joints
    const Eigen::Vector3d& contactForce() const { return Fc_; }     // world frame
    const Eigen::VectorXd& accelerations() const { return q_ddot_; } // full nv, for logging

 private:
    // dimensions
    int nv_;        // generalised coordinates (6 base + 6 joints = 12)
    int n_act_;     // actuated joints (nv_ - 6)
    int n_vars_;    // nv_ + 3   (q_ddot and one contact force)
    int n_cons_;    // 9         (6 unactuated base rows + 3 contact rows)
    double reg_;

    // the QP, assembled fresh each tick
    Eigen::MatrixXd P_;   // n_vars_ x n_vars_, cost/task matrix
    Eigen::VectorXd q_;   // n_vars_            Shifts the preferred solution towards this target
    Eigen::MatrixXd A_;   // n_cons_ x n_vars_, constraint matrix
    Eigen::VectorXd l_;   // n_cons_            Lower bound for constraints
    Eigen::VectorXd u_;   // n_cons_            Upper bound for constraints

    // cached for torque recovery, which happens after solve()
    Eigen::MatrixXd M_;   // nv_ x nv_
    Eigen::VectorXd h_;   // nv_
    Eigen::MatrixXd Jc_;  // 3 x nv_

    // results
    Eigen::VectorXd q_ddot_;
    Eigen::VectorXd tau_;
    Eigen::Vector3d Fc_;
    Status status_ = Status::kNotSolved;

    // The only OSQP resource the object owns. The workspace and data are local to
    // solve(), which builds and tears them down every tick (~73 us, 7% of a 1 kHz
    // budget). Reusing a factorisation instead is ~1.6x faster and much more code.
    OSQPSettings* settings_ = nullptr;
};

}  // namespace control
