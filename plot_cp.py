#!/usr/bin/env python3
"""Plot the capture point (xi) logged by cp_node.

Reads cp_log.csv with columns:
  time, x, y      (x,y = capture point xi in the stance-foot frame)

Left panel:  xi_x and xi_y vs time.
Right panel: xi_y vs xi_x (the capture-point path in the ground plane).

Usage:
    python3 plot_cp.py [path/to/cp_log.csv]
"""
import sys

import pandas as pd
import matplotlib.pyplot as plt

csv_path = sys.argv[1] if len(sys.argv) > 1 else "/home/nate/legs_ws/cp_log.csv"
df = pd.read_csv(csv_path)

t = df["time"]
xi_x = df["x"]
xi_y = df["y"]

fig, (ax_t, ax_xy) = plt.subplots(1, 2, figsize=(14, 6))

# --- time series ---
ax_t.plot(t, xi_x, label="xi_x (fore-aft)")
ax_t.plot(t, xi_y, label="xi_y (lateral)")
ax_t.set_xlabel("time [s]")
ax_t.set_ylabel("capture point [m]")
ax_t.set_title("Capture point vs time (stance frame)")
ax_t.grid(True)
ax_t.legend()

# --- 2D path (colored by time so you can see direction) ---
sc = ax_xy.scatter(xi_x, xi_y, c=t, cmap="viridis", s=8)
ax_xy.plot(xi_x, xi_y, color="0.7", linewidth=0.5, zorder=0)
ax_xy.scatter([0], [0], color="red", marker="x", s=80, label="stance foot")
ax_xy.set_xlabel("xi_x [m]")
ax_xy.set_ylabel("xi_y [m]")
ax_xy.set_title("Capture-point path (relative to stance foot)")
ax_xy.set_aspect("equal", adjustable="datalim")
ax_xy.grid(True)
ax_xy.legend()
fig.colorbar(sc, ax=ax_xy, label="time [s]")

fig.tight_layout()
plt.show()
