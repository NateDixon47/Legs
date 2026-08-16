#!/usr/bin/env python3
"""Plot capture-point placement and swing-foot tracking, with stance-switch markers.

Reads step_log with columns:
  time, xi_x, xi_y, step_x, step_y,
  des_x, des_y, des_z, act_x, act_y, act_z, err_vert, switched
    xi      = capture point (stance frame)
    step    = commanded foothold (stance frame)
    des     = desired foot / foothold (world), act = actual swing foot (world)
    err_vert= act_z - foothold_z  (the signed vertical error the switch gates on)
    switched= 1 on the tick a stance switch happens, else 0

Six panels:
  1. fore-aft  xi_x vs step_x       (placement: is the foothold at the capture point?)
  2. lateral   xi_y vs step_y
  3. fore-aft  des_x vs act_x       (tracking: does the foot reach the foothold?)
  4. lateral   des_y vs act_y
  5. vertical  des_z vs act_z       (does the swing foot actually come down to touchdown?)
  6. err_vert with the +/-0.005 m switch threshold shaded
Vertical dashed lines mark stance switches in all panels.

Usage:
    python3 plot_step.py [path/to/step_log]
"""
import sys

import pandas as pd
import matplotlib.pyplot as plt

# Signed vertical error band the `reached` switch requires (err_vert < 0.005).
ERR_VERT_THRESH = 0.005

path = sys.argv[1] if len(sys.argv) > 1 else "/home/nate/legs_ws/step_log"
df = pd.read_csv(path)
t = df["time"]

switch_times = df.loc[df["switched"] == 1, "time"].tolist()

fig, axes = plt.subplots(3, 2, figsize=(15, 12), sharex=True)
ax_xi_x, ax_xi_y, ax_tr_x, ax_tr_y, ax_tr_z, ax_ev = axes.flatten()

# --- placement: capture point vs commanded foothold ---
ax_xi_x.plot(t, df["xi_x"],  label="xi")
ax_xi_x.plot(t, df["step_x"], "--", label="step")
ax_xi_x.set_title("fore-aft: xi vs step")
ax_xi_x.set_ylabel("x [m]")

ax_xi_y.plot(t, df["xi_y"],  label="xi")
ax_xi_y.plot(t, df["step_y"], "--", label="step")
ax_xi_y.set_title("lateral: xi vs step")
ax_xi_y.set_ylabel("y [m]")

# --- tracking: desired foot vs actual foot ---
ax_tr_x.plot(t, df["des_x"], label="desired")
ax_tr_x.plot(t, df["act_x"], "--", label="actual")
ax_tr_x.set_title("fore-aft: desired vs actual")
ax_tr_x.set_ylabel("x [m]")

ax_tr_y.plot(t, df["des_y"], label="desired")
ax_tr_y.plot(t, df["act_y"], "--", label="actual")
ax_tr_y.set_title("lateral: desired vs actual")
ax_tr_y.set_ylabel("y [m]")

# --- vertical tracking: does the foot descend to touchdown? ---
ax_tr_z.plot(t, df["des_z"], label="desired")
ax_tr_z.plot(t, df["act_z"], "--", label="actual")
ax_tr_z.set_title("vertical: desired vs actual (swing height)")
ax_tr_z.set_ylabel("z [m]"); ax_tr_z.set_xlabel("time [s]")

# --- signed vertical error vs the switch threshold ---
ax_ev.plot(t, df["err_vert"], color="C3", label="err_vert = act_z - foothold_z")
ax_ev.axhline(ERR_VERT_THRESH, color="0.4", ls=":", lw=1.0,
              label=f"switch gate (< {ERR_VERT_THRESH} m)")
ax_ev.axhspan(df["err_vert"].min() - 0.005, ERR_VERT_THRESH,
              color="green", alpha=0.06)  # region where the vertical gate is satisfied
ax_ev.set_title("vertical error vs switch gate")
ax_ev.set_ylabel("err_vert [m]"); ax_ev.set_xlabel("time [s]")

# --- stance-switch markers on every panel ---
for ax in (ax_xi_x, ax_xi_y, ax_tr_x, ax_tr_y, ax_tr_z, ax_ev):
    for i, ts in enumerate(switch_times):
        ax.axvline(ts, color="0.25", lw=0.8,
                   label="stance switch" if i == 0 else None)
    ax.grid(True)
    ax.legend(fontsize=8)

fig.suptitle("Capture-point placement & swing tracking (dotted = stance switch)")
fig.tight_layout()
plt.show()
