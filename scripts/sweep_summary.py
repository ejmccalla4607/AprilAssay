#!/usr/bin/env python3
"""
Summarize a sweep telemetry CSV produced by sweep.sh.
Usage: python3 scripts/sweep_summary.py logs/sweep_YYYYMMDD_HHMMSS_telemetry.csv
"""
import sys
import pandas as pd

if len(sys.argv) < 2:
    print(f"Usage: {sys.argv[0]} <telemetry.csv>")
    sys.exit(1)

df = pd.read_csv(sys.argv[1])
print(f"{len(df)} frames loaded from {sys.argv[1]}\n")

header = (f"{'gain_db':>7} {'exp_us':>7} {'frames':>7} {'det':>5} "
          f"{'rate%':>6} {'margin':>7} {'W_dn':>6} {'B_dn':>6} {'sat':>4}")
print(header)
print("-" * len(header))

for (exp_us, gain_db), g in df.groupby(["exposure_us", "gain_db"]):
    n    = len(g)
    nd   = int(g["detected"].sum())
    d    = g[g["detected"] == 1]
    rate = 100.0 * nd / n if n else 0.0

    margin = d["decision_margin"].mean() if len(d) else float("nan")
    w_dn   = d["white_mean_dn"].mean()   if len(d) else float("nan")
    b_dn   = d["black_mean_dn"].mean()   if len(d) else float("nan")
    sat    = int((d["saturated"] == 1).sum()) if len(d) else 0

    print(f"{gain_db:>7.0f} {int(exp_us):>7} {n:>7} {nd:>5} {rate:>6.1f} "
          f"{margin:>7.1f} {w_dn:>6.1f} {b_dn:>6.1f} {sat:>4}")
