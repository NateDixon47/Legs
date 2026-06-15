# RobotModel implementation plan (MuJoCo backend)

`legs_dynamics` — pure C++/Eigen, **no `rclcpp`**. Depends on `utilities` (for `Side`,
`RobotState`, and the `Leg::FK` cross-check) and `mujoco_vendor` (target `mujoco::mujoco`).

## Core idea
`RobotModel` is a **query engine**, not the simulator. It owns its own `mjModel*` + `mjData*`
loaded from the MJCF, separate from the running sim. You hand it a state, it answers dynamics
questions *about that state*:

```
setState(q, q̇)  ──►  one mj_forward()  ──►  caches qM, qfrc_bias, subtree_com, kinematics
                                              │
   massMatrix() ─ biasForces() ─ comPosition() ─ footJacobian() ─ footPose()  (cheap reads)
```

## Floating-base dimensions (shapes everything)
Base is a `freejoint`, so:

| | dim | layout |
|---|---|---|
| `nq` (positions)  | **13** | base: 3 pos + 4 quat (w-first), then 6 joints |
| `nv` (velocities) | **12** | base: 3 linear + 3 angular, then 6 joints |
| `nu` (actuators)  | **6**  | the 6 joints only |

Mass matrix **M is 12×12**, bias `h` is length 12, Jacobians are **3×12**. The first 6 rows are
the *unactuated* base — that underactuation is exactly why balance needs dynamics.

---

## Data members (private)
```cpp
mjModel* model_ = nullptr;     // owned
mjData*  data_  = nullptr;     // owned
int base_body_id_;             // mj_name2id(..., mjOBJ_BODY, "base")
int left_foot_site_id_;        // mj_name2id(..., mjOBJ_SITE, "left_foot")
int right_foot_site_id_;
```
Resolve the ids **once** in the ctor; never do name lookups per query.

## Constructor / destructor / copy
- **Ctor** `RobotModel(const std::string& mjcf_path)`:
  1. `model_ = mj_loadXML(path.c_str(), nullptr, errbuf, sizeof(errbuf));` → if null, `throw std::runtime_error(errbuf)`.
  2. `data_ = mj_makeData(model_);`
  3. Resolve the 3 ids; if any `== -1`, throw (MJCF missing the body/site).
- **Dtor**: `mj_deleteData(data_); mj_deleteModel(model_);` (null-guard).
- **Copy**: `= delete` copy ctor + copy-assign (owns raw pointers; copying would double-free).

## Dimensions — trivial `const` getters
`nq()→model_->nq` (13), `nv()→model_->nv` (12), `nu()→model_->nu` (6).

## setState — the only method that does work
**`setState(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel)`** (raw, for tests):
- copy into `data_->qpos` (nq) and `data_->qvel` (nv), then `mj_forward(model_, data_);`
- that one call populates `qM`, `qfrc_bias`, `subtree_com`, `site_xpos/xmat`.

**`setState(const RobotState&)`** (the real one — packs our state into MuJoCo layout):
- `qpos[0..2]` = base position; `qpos[3..6]` = base quaternion **w,x,y,z**
  (⚠️ Eigen `quat.coeffs()` is x,y,z,w → reorder).
- joints: use `model_->jnt_qposadr[jid]` / `model_->jnt_dofadr[jid]` per joint — don't hardcode indices.
- `qvel[0..2]` base linear, `[3..5]` base angular, then joint velocities.
- ⚠️ **Verify the freejoint qvel frame convention** before trusting base velocity. Early on, feed
  base state from MuJoCo ground truth to sidestep it.
- then delegate to the raw overload.

## Dynamics getters — all `const`, pure reads of cached `data_`
| method | implementation |
|---|---|
| `massMatrix()` → 12×12 | `MatrixXd M(nv,nv); mj_fullM(model_, M.data(), data_->qM);` (symmetric → major-order irrelevant) |
| `biasForces()` → 12 | `Map<VectorXd>(data_->qfrc_bias, nv)` = **C(q,q̇)q̇ + g(q)** |
| `gravityForces()` → 12 | g(q) = bias at q̇=0. Cache in `setState` via a zero-velocity pass. **Defer until the WBC needs it.** |

## Kinematic / contact getters — `const`
| method | implementation |
|---|---|
| `comPosition()` → Vec3 | `Map<Vector3d>(data_->subtree_com + 3*base_body_id_)` |
| `footPose(Side)` → Isometry3d | pos = `Map<Vector3d>(data_->site_xpos + 3*sid)`; rot = `Map<Matrix3d,RowMajor>(data_->site_xmat + 9*sid)` |
| `footJacobian(Side)` → 3×12 | `Matrix<double,3,Dynamic,RowMajor> J(3,nv); mj_jacSite(model_, data_, J.data(), nullptr, sid);` (nullptr = skip rotational) |

⚠️ `site_xmat` and the jacobian are **row-major** — map as `RowMajor` or you get a transposed matrix.

## MJCF edit (do FIRST — Jacobians need a named point)
Add a site at each foot sole, inside the tibia bodies, co-located with the foot contact geom:
```xml
<site name="left_foot"  pos="<sole point>" size="0.01"/>
<site name="right_foot" pos="<sole point>" size="0.01"/>
```

## Proposed interface
```cpp
namespace dynamics {
class RobotModel {
 public:
  explicit RobotModel(const std::string& mjcf_path);
  ~RobotModel();
  RobotModel(const RobotModel&) = delete;
  RobotModel& operator=(const RobotModel&) = delete;

  int nq() const; int nv() const; int nu() const;

  void setState(const RobotState&);
  void setState(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel);

  Eigen::MatrixXd massMatrix()    const;  // 12x12
  Eigen::VectorXd biasForces()    const;  // 12
  Eigen::VectorXd gravityForces() const;  // 12 (later)

  Eigen::Vector3d   comPosition()      const;
  Eigen::Isometry3d footPose(Side)     const;
  Eigen::MatrixXd   footJacobian(Side) const;  // 3x12
};
}
```

---

## Build order (test gate at each step)
1. **Skeleton** — empty class, `add_library` links `mujoco::mujoco`, package builds.
2. **Ctor/dtor + dims** — test: loads MJCF, prints `nq=13 nv=12 nu=6`.
3. **setState(raw) + massMatrix + biasForces** — test: M symmetric + PD (Eigen `LLT` succeeds), sizes.
4. **MJCF sites + comPosition + footPose + footJacobian** — test: **`footPose(side)` == `Leg::FK` ∘ base pose** (model-correctness cross-check).
5. **setState(RobotState) packing + gravityForces**.
6. **footJacobian vs finite-difference of footPose** — self-check.

## Gotcha checklist
- `mj_loadXML` null-check with the error buffer.
- Eigen quat (x,y,z,w) → MuJoCo qpos (w,x,y,z) reorder.
- Row-major maps for `site_xmat` and `mj_jacSite`.
- Delete the copy constructor (owns raw pointers).
- Joint packing via `jnt_qposadr`/`jnt_dofadr`, not hardcoded indices.
- `subtree_com` / `site_xpos` indexed by `id × 3`.

## Validation philosophy
- M: 12×12, symmetric, positive-definite.
- `footPose == Leg::FK`: proves the MJCF dynamics model and your validated kinematics are the
  same robot. If they disagree, the **model** is wrong, not your controller.
- Jacobian vs finite-difference: same self-check style as the leg Jacobian.
- `gravityForces` sanity: total mass × g matches.
