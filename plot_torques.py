#!/usr/bin/env python3
"""Plot the 6 joint torques logged by torque_node as a 2x3 grid.

Top row = left leg, bottom row = right leg (yaw, pitch, knee).

Usage:
    python3 plot_torques.py [path/to/torque_log.csv]
"""
import sys

import pandas as pd
import matplotlib.pyplot as plt

csv_path = sys.argv[1] if len(sys.argv) > 1 else "/home/nate/legs_ws/torque_log.csv"
df = pd.read_csv(csv_path)

# Assume the first column is time.
time = df[df.columns[0]]

# Preferred torque column names (match torque_node's joint order). If the header
# uses something else (e.g. tau0..tau5), fall back to the last 6 columns.
preferred = ["left_hip_yaw", "left_hip_pitch", "left_knee",
             "right_hip_yaw", "right_hip_pitch", "right_knee"]
cols = preferred if all(c in df.columns for c in preferred) else list(df.columns[-6:])

# 2 rows x 3 cols; share x (time) and y (torque) so scales are comparable.
fig, axes = plt.subplots(2, 3, figsize=(14, 7), sharex=True, sharey=True)

# axes is a 2x3 array; flatten() gives a 1-D view so we can zip it with the columns.
for ax, col in zip(axes.flatten(), cols):
    ax.plot(time, df[col])
    ax.set_title(col)
    ax.grid(True)

# Label only the outer edges to avoid clutter.
for ax in axes[-1]:            # bottom row
    ax.set_xlabel("time [s]")
for ax in axes[:, 0]:          # left column
    ax.set_ylabel("torque [N·m]")

fig.suptitle("Joint torques")
fig.tight_layout()
plt.show()
