#!/usr/bin/env python3
"""Plot right-leg joint angles: actual vs desired, one subplot per joint.

Reads q_log.csv with columns:
  time, right_hip_yaw, right_hip_pitch, right_knee,
        right_hip_yaw_d, right_hip_pitch_d, right_knee_d

Usage:
    python3 plot_q.py [path/to/q_log.csv]
"""
import sys

import pandas as pd
import matplotlib.pyplot as plt

csv_path = sys.argv[1] if len(sys.argv) > 1 else "/home/nate/legs_ws/q_log.csv"
df = pd.read_csv(csv_path)

time = df[df.columns[0]]

# (actual column, desired column, title) for each right-leg joint.
joints = [
    ("right_hip_yaw",   "right_hip_yaw_d",   "right_hip_yaw"),
    ("right_hip_pitch", "right_hip_pitch_d", "right_hip_pitch"),
    ("right_knee",      "right_knee_d",      "right_knee"),
]

# 1 row x 3 cols; share x so the time axes line up. axes is a 1-D array here.
fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharex=True)

for ax, (actual, desired, title) in zip(axes, joints):
    ax.plot(time, df[actual],  label="actual")
    ax.plot(time, df[desired], "--", label="desired")
    ax.set_title(title)
    ax.set_xlabel("time [s]")
    ax.grid(True)
    ax.legend()

axes[0].set_ylabel("q [rad]")
fig.suptitle("Right leg: actual vs desired joint angles")
fig.tight_layout()
plt.show()
