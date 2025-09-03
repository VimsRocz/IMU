#!/usr/bin/env python3
"""
Parse logs/* for lines that start with [SUMMARY] and emit:
  summary.csv
  summary.md
"""

import csv
import pathlib
import re
import os
import logging
from naming import results_path

logging.basicConfig(level=logging.INFO, format="%(message)s")
if __name__ == "__main__":
    os.makedirs('results', exist_ok=True)
    logging.info("Ensured 'results/' directory exists.")
RESULTS_DIR = pathlib.Path("results")

LOG_DIR = pathlib.Path("logs")
SUMMARY = re.compile(r"\[SUMMARY\]\s+(.*)")

rows = {}
for log in LOG_DIR.glob("*.log"):
    for line in log.read_text().splitlines():
        m = SUMMARY.search(line)
        if m:
            pairs = re.findall(r"(\w+)=\s*([^\s]+)", m.group(1))
            kv = {k: v for k, v in pairs}
            key = (kv.get("imu"), kv.get("method"))
            rows[key] = kv

rows = list(rows.values())

# normalise values and compute dataset name
for r in rows:
    r["dataset"] = pathlib.Path(r["imu"]).stem
    r["rmse_pos"] = float(r["rmse_pos"].replace("m", ""))
    r["final_pos"] = float(r["final_pos"].replace("m", ""))
    if "rms_resid_pos" in r:
        r["rms_resid_pos"] = float(r["rms_resid_pos"].replace("m", ""))
    if "rms_resid_vel" in r:
        r["rms_resid_vel"] = float(r["rms_resid_vel"].replace("m", ""))
    if "max_resid_pos" in r:
        r["max_resid_pos"] = float(r["max_resid_pos"].replace("m", ""))
    if "max_resid_vel" in r:
        r["max_resid_vel"] = float(r["max_resid_vel"].replace("m", ""))
    if "att_err_deg" in r:
        r["att_err_deg"] = float(r["att_err_deg"])
    if "q_final" in r:
        q = r["q_final"].strip("[]").split(",")
        if len(q) == 4:
            r["qw"], r["qx"], r["qy"], r["qz"] = map(float, q)

# determine best method per dataset based on final_pos
best_map = {}
for r in rows:
    ds = r["dataset"]
    best = best_map.get(ds)
    if best is None or r["final_pos"] < best["final_pos"]:
        best_map[ds] = r
for r in rows:
    r["best"] = "\u2713" if best_map[r["dataset"]] is r else ""

# CSV -------------------------------------------------------------------------
with open(results_path("summary.csv"), "w", newline="") as fh:
    fieldnames = [
        "dataset",
        "method",
        "imu",
        "gnss",
        "rmse_pos",
        "final_pos",
        "rms_resid_pos",
        "max_resid_pos",
        "rms_resid_vel",
        "max_resid_vel",
        "att_err_deg",
        "qw",
        "qx",
        "qy",
        "qz",
        "best",
    ]
    writer = csv.DictWriter(fh, fieldnames=fieldnames)
    writer.writeheader()
    for r in rows:
        writer.writerow({k: r.get(k, "") for k in fieldnames})

# Markdown table --------------------------------------------------------------
with open(results_path("summary.md"), "w") as fh:
    hdr_cols = [
        "dataset",
        "method",
        "imu",
        "gnss",
        "rmse_pos",
        "final_pos",
        "rms_resid_pos",
        "max_resid_pos",
        "rms_resid_vel",
        "max_resid_vel",
        "att_err_deg",
        "qw",
        "qx",
        "qy",
        "qz",
        "best",
    ]
    hdr = " | ".join(hdr_cols)
    sep = " | ".join("---" for _ in hdr_cols)
    fh.write(hdr + "\n" + sep + "\n")
    for r in rows:
        fh.write(
            f"{r['dataset']} | {r['method']} | {r['imu']} | {r['gnss']} | "
            f"{r['rmse_pos']:.2f} | {r['final_pos']:.2f} | {r.get('rms_resid_pos', float('nan')):.2f} | "
            f"{r.get('max_resid_pos', float('nan')):.2f} | {r.get('rms_resid_vel', float('nan')):.2f} | "
            f"{r.get('max_resid_vel', float('nan')):.2f} | {r.get('att_err_deg', float('nan')):.2f} | "
            f"{r.get('qw', float('nan')):.6f} | {r.get('qx', float('nan')):.6f} | {r.get('qy', float('nan')):.6f} | {r.get('qz', float('nan')):.6f} | {r['best']}\n"
        )

print("Created results/summary.csv and results/summary.md")
