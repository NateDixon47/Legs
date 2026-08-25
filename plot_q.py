#!/usr/bin/env python3
"""Plot both legs' joint angles, colored by stance/swing phase.

Reads q_log.csv with columns:
  time, stance,
  left_hip_yaw, left_hip_pitch, left_knee, right_hip_yaw, right_hip_pitch, right_knee

  stance column: 0 = LEFT leg is the stance leg, 1 = RIGHT leg is the stance leg.

Each joint's angle is drawn in the STANCE color while that leg is the stance leg
and the SWING color while it is swinging.

There is no "desired" trace any more: the controller is task-space, so there is no
desired joint angle to compare against. Swing tracking is a Cartesian quantity --
see plot_step.py for desired-vs-actual foot position.

Usage:
    python3 plot_q.py [path/to/q_log.csv]
"""
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

csv_path = sys.argv[1] if len(sys.argv) > 1 else "/home/nate/legs_ws/q_log.csv"
df = pd.read_csv(csv_path)

time = df["time"] - df["time"].iloc[0]
stance = df["stance"].to_numpy()   # 0 = left stance, 1 = right stance

STANCE_C = "tab:blue"
SWING_C = "tab:orange"

# (column, which leg) — left leg top row, right leg bottom row.
joints = [
    ("left_hip_yaw",    "left"),
    ("left_hip_pitch",  "left"),
    ("left_knee",       "left"),
    ("right_hip_yaw",   "right"),
    ("right_hip_pitch", "right"),
    ("right_knee",      "right"),
]

fig, axes = plt.subplots(2, 3, figsize=(16, 8), sharex=True)

for ax, (col, side) in zip(axes.flatten(), joints):
    a = df[col].to_numpy()
    # Is THIS leg the stance leg at each sample?
    is_stance = (stance == 0) if side == "left" else (stance == 1)

    # Split the series into stance/swing by masking the other phase to NaN
    # (NaN breaks the line, so each phase draws only over its own samples).
    ax.plot(time, np.where(is_stance, a, np.nan), color=STANCE_C, label="stance")
    ax.plot(time, np.where(~is_stance, a, np.nan), color=SWING_C, label="swing")
    ax.set_title(col)
    ax.grid(True)

axes[0, 0].set_ylabel("q [rad]")
axes[1, 0].set_ylabel("q [rad]")
for ax in axes[1]:
    ax.set_xlabel("time [s]")
axes[0, 0].legend(fontsize=8)

fig.suptitle("Joint angles, colored by stance / swing phase")
fig.tight_layout()
plt.show()
