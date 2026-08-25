#!/usr/bin/env python3
"""Plot capture-point placement and swing-foot tracking.

Reads TWO logs, since the pipeline split into separate nodes:

  cp_log.csv    (cp_node, 40 Hz)   time, xi_x, xi_y, step_x, step_y
      xi   = capture point, stance-foot origin / world-aligned axes
      step = commanded foothold, same frame

  traj_log.csv  (traj_node, 500 Hz)
      time, stance, t_swing, tau, contact_l, contact_r, has_lifted,
      step_x, step_y, des_x, des_y, des_z, vz_des, act_x, act_y, act_z, switched
      des  = commanded swing-foot position (world)
      act  = measured swing-foot position (world)

Five panels:
  1. fore-aft  xi_x vs step_x     placement: is the foothold at the capture point?
  2. lateral   xi_y vs step_y
  3. fore-aft  des_x vs act_x, with `origin` (= des_x - step_x = stance foot x)
     overlaid. The origin steps at a stance switch while step_x stays frozen until
     cp publishes again -- that gap is the stale-foothold double count.
  4. lateral   des_y vs act_y
  5. vertical  des_z vs act_z
Vertical dashed lines mark stance switches in all panels.

Usage:
    python3 plot_step.py [traj_log.csv] [cp_log.csv]
"""
import sys
import os

import pandas as pd
import matplotlib.pyplot as plt

traj_path = sys.argv[1] if len(sys.argv) > 1 else "/home/nate/legs_ws/traj_log.csv"
cp_path = sys.argv[2] if len(sys.argv) > 2 else "/home/nate/legs_ws/cp_log.csv"

if not os.path.exists(traj_path):
    sys.exit(f"missing {traj_path} -- run the sim first (traj_node writes it)")

tj = pd.read_csv(traj_path)
cp = pd.read_csv(cp_path) if os.path.exists(cp_path) else None

# The two nodes stamp from the same clock but start at different instants; zero both
# against the earlier start so the panels line up on one axis.
t0 = tj["time"].iloc[0]
if cp is not None and len(cp):
    t0 = min(t0, cp["time"].iloc[0])
tj_t = tj["time"] - t0
cp_t = (cp["time"] - t0) if cp is not None else None

switch_times = (tj.loc[tj["switched"] == 1, "time"] - t0).tolist()

fig, axes = plt.subplots(3, 2, figsize=(15, 12), sharex=True)
ax_xi_x, ax_xi_y, ax_tr_x, ax_tr_y, ax_tr_z, ax_unused = axes.flatten()
ax_unused.axis("off")
panels = (ax_xi_x, ax_xi_y, ax_tr_x, ax_tr_y, ax_tr_z)

# --- placement: capture point vs commanded foothold (cp_node) ---
if cp is not None and len(cp):
    ax_xi_x.plot(cp_t, cp["xi_x"], label="xi")
    ax_xi_x.plot(cp_t, cp["step_x"], "--", label="step")
    ax_xi_y.plot(cp_t, cp["xi_y"], label="xi")
    ax_xi_y.plot(cp_t, cp["step_y"], "--", label="step")
else:
    for ax in (ax_xi_x, ax_xi_y):
        ax.text(0.5, 0.5, f"no {os.path.basename(cp_path)}",
                ha="center", va="center", transform=ax.transAxes)
ax_xi_x.set_title("fore-aft: xi vs step (stance frame)")
ax_xi_x.set_ylabel("x [m]")
ax_xi_y.set_title("lateral: xi vs step (stance frame)")
ax_xi_y.set_ylabel("y [m]")

# --- tracking: desired foot vs actual foot (traj_node) ---
# `origin` is the stance foot the foothold is measured from. It steps at a switch;
# if des_x steps with it while step_x is still the pre-switch value, that jump is
# the old foothold added to the new stance foot.
origin_x = tj["des_x"] - tj["step_x"]
ax_tr_x.plot(tj_t, tj["des_x"], label="desired")
ax_tr_x.plot(tj_t, tj["act_x"], "--", label="actual")
ax_tr_x.plot(tj_t, origin_x, ":", color="0.5", lw=1.2, label="origin (stance foot x)")
ax_tr_x.set_title("fore-aft: desired vs actual (world)")
ax_tr_x.set_ylabel("x [m]")

ax_tr_y.plot(tj_t, tj["des_y"], label="desired")
ax_tr_y.plot(tj_t, tj["act_y"], "--", label="actual")
ax_tr_y.set_title("lateral: desired vs actual (world)")
ax_tr_y.set_ylabel("y [m]")

ax_tr_z.plot(tj_t, tj["des_z"], label="desired")
ax_tr_z.plot(tj_t, tj["act_z"], "--", label="actual")
ax_tr_z.set_title("vertical: desired vs actual (swing height)")
ax_tr_z.set_ylabel("z [m]")

for ax in (ax_tr_y, ax_tr_z):
    ax.set_xlabel("time [s]")

# --- stance-switch markers on every panel ---
for ax in panels:
    for i, ts in enumerate(switch_times):
        ax.axvline(ts, color="0.25", lw=0.8,
                   label="stance switch" if i == 0 else None)
    ax.grid(True)
    ax.legend(fontsize=8)

fig.suptitle("Capture-point placement & swing tracking "
             "(vertical lines = stance switch)")
fig.tight_layout()
plt.show()
