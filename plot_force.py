#!/usr/bin/env python3
"""Plot the stance-foot ground-reaction force from the controller's force_log.csv.

Columns: time, Fx, Fy, Fz
    Fx, Fy = horizontal restoring force (ankle/CoP strategy)
    Fz     = vertical support force (weight + height controller)

Two panels:
  1. Fx, Fy, Fz vs time                 (the three force components)
  2. |F_xy| vs the friction-cone limit  (mu*Fz) — is the horizontal force
     saturating the cone? If |F_xy| rides the mu*Fz line, the foot is at the
     slip/tip limit and the horizontal gains can't do more.

Usage:
    python3 plot_force.py [path/to/force_log.csv] [mu]
"""
import sys

import pandas as pd
import matplotlib.pyplot as plt

path = sys.argv[1] if len(sys.argv) > 1 else "/home/nate/legs_ws/force_log.csv"
mu = float(sys.argv[2]) if len(sys.argv) > 2 else 0.8

# Robust read: tolerate a header that ran into the first data row (missing \n).
df = pd.read_csv(path)
if "Fz" not in df.columns:
    # Header merged with first row, e.g. last column named "Fz0.002000".
    df = pd.read_csv(path, header=None, names=["time", "Fx", "Fy", "Fz"],
                     skiprows=1)
    df = df.apply(pd.to_numeric, errors="coerce").dropna()

t = df["time"]
f_xy = (df["Fx"] ** 2 + df["Fy"] ** 2) ** 0.5
cone = mu * df["Fz"].clip(lower=0.0)

fig, (ax_f, ax_c) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

ax_f.plot(t, df["Fx"], label="Fx (fore-aft)")
ax_f.plot(t, df["Fy"], label="Fy (lateral)")
ax_f.plot(t, df["Fz"], label="Fz (vertical)")
ax_f.axhline(0.0, color="0.7", lw=0.8)
ax_f.set_ylabel("force [N]")
ax_f.set_ylim(-400, 400)
ax_f.set_title("Stance-foot ground-reaction force")
ax_f.grid(True); ax_f.legend(fontsize=8)

ax_c.plot(t, f_xy, label="|F_xy|")
ax_c.plot(t, cone, "--", color="r", label=f"friction cone (mu*Fz, mu={mu})")
ax_c.set_ylabel("horizontal force [N]"); ax_c.set_xlabel("time [s]")
ax_c.set_ylim(0, 400)
ax_c.set_title("Horizontal force vs friction-cone limit")
ax_c.grid(True); ax_c.legend(fontsize=8)

fig.tight_layout()
plt.show()
