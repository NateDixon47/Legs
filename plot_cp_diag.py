#!/usr/bin/env python3
"""Diagnose capture-point divergence: decompose xi and validate comVelocity.

Reads cp_diag.csv with columns:
  time, x_x, x_y, xdot_x, xdot_y, xi_x, xi_y, fd_x, fd_y
    x       = CoM position relative to the stance foot
    xdot    = comVelocity() (the x_dot used in xi = x + xdot/omega)
    xi      = capture point
    fd      = finite-difference of comPosition() (should ~= xdot if comVelocity is correct)

Top row:    xi vs x per axis  -> if xi runs away while x stays small, the xdot/omega term is the culprit.
Bottom row: xdot vs fd per axis -> if they disagree, comVelocity() has a bug (frame/scale).

Usage:
    python3 plot_cp_diag.py [path/to/cp_diag.csv]
"""
import sys

import pandas as pd
import matplotlib.pyplot as plt

csv_path = sys.argv[1] if len(sys.argv) > 1 else "/home/nate/legs_ws/cp_diag.csv"
df = pd.read_csv(csv_path)
t = df["time"]

fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)

# --- top row: xi vs x (decomposition) ---
axes[0, 0].plot(t, df["x_x"],  label="x (CoM-stance)")
axes[0, 0].plot(t, df["xi_x"], label="xi (capture pt)")
axes[0, 0].set_title("fore-aft: x vs xi")
axes[0, 0].set_ylabel("[m]")

axes[0, 1].plot(t, df["x_y"],  label="x (CoM-stance)")
axes[0, 1].plot(t, df["xi_y"], label="xi (capture pt)")
axes[0, 1].set_title("lateral: x vs xi")

# --- bottom row: comVelocity vs finite-difference (validation) ---
axes[1, 0].plot(t, df["xdot_x"], label="comVelocity")
axes[1, 0].plot(t, df["fd_x"], "--", label="finite-diff")
axes[1, 0].set_title("fore-aft: x_dot vs finite-diff")
axes[1, 0].set_ylabel("[m/s]")
axes[1, 0].set_xlabel("time [s]")

axes[1, 1].plot(t, df["xdot_y"], label="comVelocity")
axes[1, 1].plot(t, df["fd_y"], "--", label="finite-diff")
axes[1, 1].set_title("lateral: x_dot vs finite-diff")
axes[1, 1].set_xlabel("time [s]")

for ax in axes.flatten():
    ax.grid(True)
    ax.legend(fontsize=8)

fig.suptitle("Capture-point diagnostics")
fig.tight_layout()
plt.show()
