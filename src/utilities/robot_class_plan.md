# Robot Class — Design Plan

Planning notes for the `Robot` class (lives in the `utilities` / kinematics library,
namespace `legs`). Pure C++/Eigen — **no ROS**. Composes the two `Leg`s.

## The class's job
Be the **whole-leg-pair composer**. It holds both legs and the joint ordering, and it
translates between the two "languages" in the system:
- the **per-leg angle** view (`LegAngles`), and
- the **flat 6-number** view used by the ROS controller (`/position_controller/commands`)
  and `/joint_states`.

## What it holds (state)
- **The two legs** — a left `Leg` and a right `Leg`.
- **The joint ordering** — the six joint names in the exact order the controller expects:
  `left_hip_yaw, left_hip_pitch, left_knee, right_hip_yaw, right_hip_pitch, right_knee`.
  This is the **single source of truth** for the joint layout — nothing else in the system
  should hardcode that order (it replaces the hand-built `{…, 0, 0, 0}` padding in the node).

## Functionality to implement now

1. **Construction** — build both legs (left/right) and set up the joint ordering. After
   construction the object is ready to use; no further setup needed.

2. **Leg access** — return a reference to a chosen leg (by `Side`) so a caller can use that
   leg's `FK`/`IK`/limits directly when they only care about one leg. (Provide a `const` and
   non-const version.)

3. **Command generation (main feature)** — take a desired foot position for each leg, run
   each leg's `IK`, and assemble the results into the single ordered command vector the
   controller consumes. Replaces the hand-built padding in the node. Must report failure when
   a leg can't reach its target.

4. **State ingestion** — given the joint names + positions from `/joint_states`, pull out each
   leg's three joint angles **by matching names** (never by assuming array order). Output: the
   two legs' current `LegAngles`. Feeds the FK-check now, the state estimator later.

5. **Foot-position convenience (verification)** — given both legs' angles, return both foot
   positions (each via that leg's `FK`). Handy for the FK-check / monitoring. Optional but cheap.

6. **Expose the joint ordering** — so the node (or anything) can ask "what's the joint layout?"
   rather than duplicating it. Keep it the single source.

## Decisions to make before writing it
- **Failure behavior of command generation** — when only one leg's IK solves:
  - v1 (recommended): return "no command" so a half-valid command is never sent (node warns).
  - later: command the good leg and hold the other.
- **Both-legs vs per-leg commanding** — v1 can require both targets; per-leg commanding
  (move one, hold the other) can come later. Probably not needed now.
- **Stateless vs stateful** — recommended **stateless** for now: compute commands / parse
  states on demand, don't store live joint values. Holding current state is `RobotState`'s
  job later.

## Boundaries (keep out)
- No `rclcpp`, no message types, no topic names — the node converts ROS messages ↔ the plain
  types this class uses.
- No publishing/subscribing — that's the node.

## Seats for later (don't build yet)
- `center_of_mass()`
- `support_polygon()`
- base pose / `RobotState` integration
- whole-body Jacobian

`Robot` grows these once a consumer (balance / MPC) exists.

## Files
- `utilities/include/utilities/robot.hpp` — declarations
- `utilities/src/robot.cpp` — implementation
- add `src/robot.cpp` to the `kinematics` library target in `CMakeLists.txt`

## Two things to nail down first
1. The **joint ordering as the single source of truth**.
2. What **command generation does on a one-leg failure**.
