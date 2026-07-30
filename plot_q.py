#!/usr/bin/env python3
"""Plot both legs' joint angles (actual vs desired), colored by stance/swing phase.

Reads q_log.csv with columns:
  time, stance,
  left_hip_yaw, left_hip_pitch, left_knee, right_hip_yaw, right_hip_pitch, right_knee,
  left_hip_yaw_d, ... , right_knee_d           (desired, _d suffix)

  stance column: 0 = LEFT leg is the stance leg, 1 = RIGHT leg is the stance leg.

The actual-angle line is drawn in the STANCE color while that leg is the stance leg
and the SWING color while it is swinging; desired is a dashed black line.

Usage:
    python3 plot_q.py [path/to/q_log.csv]
"""
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

csv_path = sys.argv[1] if len(sys.argv) > 1 else "/home/nate/legs_ws/q_log.csv"
df = pd.read_csv(csv_path)

time = df["time"]
stance = df["stance"].to_numpy()   # 0 = left stance, 1 = right stance

STANCE_C = "tab:blue"
SWING_C = "tab:orange"

# (actual col, desired col, which leg) — left leg top row, right leg bottom row.
joints = [
    ("left_hip_yaw",    "left_hip_yaw_d",    "left"),
    ("left_hip_pitch",  "left_hip_pitch_d",  "left"),
    ("left_knee",       "left_knee_d",       "left"),
    ("right_hip_yaw",   "right_hip_yaw_d",   "right"),
    ("right_hip_pitch", "right_hip_pitch_d", "right"),
    ("right_knee",      "right_knee_d",      "right"),
]

fig, axes = plt.subplots(2, 3, figsize=(16, 8), sharex=True)

for ax, (actual, desired, side) in zip(axes.flatten(), joints):
    a = df[actual].to_numpy()
    # Is THIS leg the stance leg at each sample?
    is_stance = (stance == 0) if side == "left" else (stance == 1)

    # Split the actual series into stance/swing by masking the other phase to NaN
    # (NaN breaks the line, so each phase draws only over its own samples).
    stance_seg = np.where(is_stance, a, np.nan)
    swing_seg = np.where(~is_stance, a, np.nan)

    ax.plot(time, stance_seg, color=STANCE_C, label="actual (stance)")
    ax.plot(time, swing_seg, color=SWING_C, label="actual (swing)")
    ax.plot(time, df[desired], "--", color="k", linewidth=0.9, label="desired")
    ax.set_title(actual)
    ax.grid(True)

axes[0, 0].set_ylabel("q [rad]")
axes[1, 0].set_ylabel("q [rad]")
for ax in axes[1]:
    ax.set_xlabel("time [s]")
axes[0, 0].legend(fontsize=8)

fig.suptitle("Joint angles: actual (stance vs swing) vs desired")
fig.tight_layout()
plt.show()
