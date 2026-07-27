#!/usr/bin/env python3
"""Plot stance-leg posture control: height, torso roll/pitch/yaw, and orientation error.

Reads three CSVs from controller_node:
  height_log.csv : time, torso_height, desired_height
  rot_log.csv    : time, roll, pitch, yaw, roll_d, pitch_d, yaw_d
  posture_e.csv  : time, roll_e, pitch_e, yaw_e   (e_rot; target 0)

Produces a 2x3 grid: [height, roll, pitch, yaw, e_rot], each showing actual vs desired.

Usage:
    python3 plot_posture.py [height_log.csv] [rot_log.csv] [posture_e.csv]
"""
import sys

import pandas as pd
import matplotlib.pyplot as plt

height_path = sys.argv[1] if len(sys.argv) > 1 else "/home/nate/legs_ws/height_log.csv"
rot_path    = sys.argv[2] if len(sys.argv) > 2 else "/home/nate/legs_ws/rot_log.csv"
err_path    = sys.argv[3] if len(sys.argv) > 3 else "/home/nate/legs_ws/posture_e.csv"

hdf = pd.read_csv(height_path)
rdf = pd.read_csv(rot_path)
edf = pd.read_csv(err_path)

fig, axes = plt.subplots(2, 3, figsize=(17, 8))
ax_h, ax_roll, ax_pitch, ax_yaw, ax_err, ax_unused = axes.flatten()

# --- height ---
ax_h.plot(hdf["time"], hdf["torso_height"],   label="actual")
ax_h.plot(hdf["time"], hdf["desired_height"], "--", label="desired")
ax_h.set_title("height")
ax_h.set_ylabel("z [m]")

# --- roll / pitch / yaw: (axis, actual col, desired col, title) ---
for ax, actual, desired, title in [
    (ax_roll,  "roll",  "roll_d",  "roll"),
    (ax_pitch, "pitch", "pitch_d", "pitch"),
    (ax_yaw,   "yaw",   "yaw_d",   "yaw"),
]:
    ax.plot(rdf["time"], rdf[actual],  label="actual")
    ax.plot(rdf["time"], rdf[desired], "--", label="desired")
    ax.set_title(title)
    ax.set_ylabel("angle [rad]")

# --- orientation error e_rot (target 0) ---
ax_err.plot(edf["time"], edf["roll_e"],  label="roll_e")
ax_err.plot(edf["time"], edf["pitch_e"], label="pitch_e")
ax_err.plot(edf["time"], edf["yaw_e"],   label="yaw_e")
ax_err.axhline(0.0, color="k", linewidth=0.8, linestyle=":")  # target
ax_err.set_title("orientation error (e_rot)")
ax_err.set_ylabel("error [rad]")

for ax in (ax_h, ax_roll, ax_pitch, ax_yaw, ax_err):
    ax.set_xlabel("time [s]")
    ax.grid(True)
    ax.legend()

ax_unused.axis("off")  # 6th cell unused in the 2x3 grid

fig.suptitle("Stance posture: actual vs desired + orientation error")
fig.tight_layout()
plt.show()
