#include "legs_control/wbc.hpp"
#include <stdexcept>

namespace control {

WholeBodyController::WholeBodyController(int nv, double reg) : nv_(nv), n_act_(nv - 6), n_vars_(nv + 3), n_cons_(9), reg_(reg) {
    P_ = reg_ * Eigen::MatrixXd::Identity(n_vars_, n_vars_);
    q_ = Eigen::VectorXd::Zero(n_vars_);
    A_ = Eigen::MatrixXd::Zero(n_cons_, n_vars_);
    l_ = Eigen::VectorXd::Zero(n_cons_);
    u_ = Eigen::VectorXd::Zero(n_cons_);

    M_ = Eigen::MatrixXd::Zero(nv_, nv_);
    h_ = Eigen::VectorXd::Zero(nv_);
    Jc_ = Eigen::MatrixXd::Zero(3, nv_);

    q_ddot_ = Eigen::VectorXd::Zero(nv_);
    tau_ = Eigen::VectorXd::Zero(n_act_);
    Fc_.setZero();

    // Solver settings are fixed for the object's lifetime; the workspace itself is
    // built inside solve(). The defaults are far too loose for a controller -- OSQP
    // reports "solved" at 1e-3, which is percent-level error in the torques.
    settings_ = static_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
    osqp_set_default_settings(settings_);
    settings_->verbose = 0;
    settings_->eps_abs = 1e-8;
    settings_->eps_rel = 1e-8;
    settings_->max_iter = 20000;
    settings_->polish = 1;
}

WholeBodyController::~WholeBodyController() {
    if (settings_) c_free(settings_);
}

void WholeBodyController::reset() {
    P_ = reg_ * Eigen::MatrixXd::Identity(n_vars_, n_vars_);
    q_.setZero();
    A_.setZero();
    l_.setZero();
    u_.setZero();
    status_ = Status::kNotSolved;
}

void WholeBodyController::setDynamics(const Eigen::MatrixXd& M, const Eigen::VectorXd& h) {
    M_ = M;
    h_ = h;

    A_.block(0, 0, 6, nv_) = M.topRows(6);
    l_.head(6) = -h.head(6);
    u_.head(6) = -h.head(6);
}

void WholeBodyController::setStanceContact(const Eigen::MatrixXd& Jc, const Eigen::MatrixXd& Jc_dot, const Eigen::VectorXd& v) {
    Jc_ = Jc;
    A_.block(0, nv_, 6, 3) = -Jc.leftCols(6).transpose();

    A_.block(6, 0, 3, nv_) = Jc;
    l_.tail(3) = -Jc_dot*v;
    u_.tail(3) = -Jc_dot*v;
}

void WholeBodyController::addSwingTask(const Eigen::MatrixXd& Js, const Eigen::MatrixXd& Js_dot, const Eigen::VectorXd& v, const Eigen::Vector3d& a_des, double weight) {
    const Eigen::Vector3d b = a_des - Js_dot * v;
    Eigen::Matrix3d W = Eigen::Matrix3d::Identity() * weight;
    Eigen::MatrixXd A_swing = Eigen::MatrixXd::Zero(3, nv_);
    A_swing.block(0, 0, 3, nv_) = Js;

    P_.topLeftCorner(nv_, nv_) += A_swing.transpose() * W * A_swing;
    q_.head(nv_) += -A_swing.transpose() * W * b;
}

namespace {

// Dense -> compressed sparse column, the only format OSQP accepts.
// upper_only keeps just the upper triangle, which is what it requires for P.
csc* denseToCsc(const Eigen::MatrixXd& A, bool upper_only) {
    const c_int m = A.rows(), n = A.cols();
    c_int nnz = 0;
    for (c_int j = 0; j < n; ++j)
        for (c_int i = 0; i < m; ++i)
            if (!(upper_only && i > j)) ++nnz;

    c_float* x = static_cast<c_float*>(c_malloc(nnz * sizeof(c_float)));
    c_int* ir  = static_cast<c_int*>(c_malloc(nnz * sizeof(c_int)));
    c_int* cp  = static_cast<c_int*>(c_malloc((n + 1) * sizeof(c_int)));

    c_int k = 0;
    for (c_int j = 0; j < n; ++j) {
        cp[j] = k;                         // column j starts here
        for (c_int i = 0; i < m; ++i) {
            if (upper_only && i > j) continue;
            ir[k] = i;
            x[k]  = A(i, j);
            ++k;
        }
    }
    cp[n] = k;                             // final entry is the total count
    return csc_matrix(m, n, nnz, x, ir, cp);
}

void freeCsc(csc* M) {
    if (!M) return;
    c_free(M->x);
    c_free(M->i);
    c_free(M->p);
    c_free(M);
}

}  // namespace

WholeBodyController::Status WholeBodyController::solve() {
    OSQPData* data = static_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
    data->n = n_vars_;
    data->m = n_cons_;
    data->P = denseToCsc(P_, true);    // upper triangle only
    data->A = denseToCsc(A_, false);
    data->q = q_.data();
    data->l = l_.data();
    data->u = u_.data();

    OSQPWorkspace* work = nullptr;
    auto cleanup = [&]() {
        if (work) osqp_cleanup(work);
        freeCsc(data->P);
        freeCsc(data->A);
        c_free(data);
    };

    if (osqp_setup(&work, data, settings_) != 0) {
        cleanup();
        status_ = Status::kSolverFailed;
        return status_;
    }

    osqp_solve(work);

    // OSQP returns finite numbers even when it fails: a dual-infeasible problem
    // yields plausible-looking garbage, not NaN. This check is the only thing
    // between that and the motors.
    if (work->info->status_val != OSQP_SOLVED) {
        cleanup();
        status_ = Status::kSolverFailed;
        return status_;
    }

    Eigen::Map<const Eigen::VectorXd> x(work->solution->x, n_vars_);
    q_ddot_ = x.head(nv_);
    Fc_     = x.tail(3);

    // Bottom 6 dynamics rows: tau is fully determined once qddot and Fc are known.
    tau_ = M_.bottomRows(n_act_) * q_ddot_ + h_.tail(n_act_)
           - Jc_.rightCols(n_act_).transpose() * Fc_;

    cleanup();
    status_ = Status::kOk;
    return status_;
}

}
