#!/usr/bin/env python3
"""Plot the capture point and commanded foothold logged by cp_node.

Reads cp_log.csv with columns:
  time, xi_x, xi_y, step_x, step_y

  xi   = capture point
  step = commanded foothold from compute_px
Both are in the stance-foot frame: origin at the stance foot, axes world-aligned
(NOT rotated into the foot's frame).

Left panel:  xi and step vs time, both axes.
Right panel: the ground-plane paths of xi and step, relative to the stance foot.
             The gap between them is what compute_px adds on top of the raw
             capture point.

Usage:
    python3 plot_cp.py [path/to/cp_log.csv]
"""
import sys

import pandas as pd
import matplotlib.pyplot as plt

csv_path = sys.argv[1] if len(sys.argv) > 1 else "/home/nate/legs_ws/cp_log.csv"
df = pd.read_csv(csv_path)

t = df["time"] - df["time"].iloc[0]
xi_x, xi_y = df["xi_x"], df["xi_y"]
step_x, step_y = df["step_x"], df["step_y"]

fig, (ax_t, ax_xy) = plt.subplots(1, 2, figsize=(14, 6))

# --- time series: capture point and the foothold derived from it ---
ax_t.plot(t, xi_x, color="C0", label="xi_x (fore-aft)")
ax_t.plot(t, step_x, "--", color="C0", label="step_x")
ax_t.plot(t, xi_y, color="C1", label="xi_y (lateral)")
ax_t.plot(t, step_y, "--", color="C1", label="step_y")
ax_t.set_xlabel("time [s]")
ax_t.set_ylabel("[m], stance frame")
ax_t.set_title("Capture point vs commanded foothold")
ax_t.grid(True)
ax_t.legend(fontsize=8)

# --- 2D paths (colored by time so direction is readable) ---
sc = ax_xy.scatter(xi_x, xi_y, c=t, cmap="viridis", s=8, label="xi")
ax_xy.plot(xi_x, xi_y, color="0.7", linewidth=0.5, zorder=0)
ax_xy.plot(step_x, step_y, color="C3", linewidth=0.8, alpha=0.7, label="step")
ax_xy.scatter([0], [0], color="red", marker="x", s=80, label="stance foot")
ax_xy.set_xlabel("x [m]")
ax_xy.set_ylabel("y [m]")
ax_xy.set_title("Ground-plane path (relative to stance foot)")
ax_xy.set_aspect("equal", adjustable="datalim")
ax_xy.grid(True)
ax_xy.legend(fontsize=8)
fig.colorbar(sc, ax=ax_xy, label="time [s]")

fig.tight_layout()
plt.show()
