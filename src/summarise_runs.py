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

logging.basicConfig(level=logging.INFO, format="%(message)s")
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
with open(RESULTS_DIR / "summary.csv", "w", newline="") as fh:
    fieldnames = [
        "dataset",
        "method",
        "imu",
        "gnss",
        "rmse_pos",
        "final_pos",
        "best",
    ]
    writer = csv.DictWriter(fh, fieldnames=fieldnames)
    writer.writeheader()
    for r in rows:
        writer.writerow({k: r.get(k, "") for k in fieldnames})

# Markdown table --------------------------------------------------------------
with open(RESULTS_DIR / "summary.md", "w") as fh:
    hdr_cols = ["dataset", "method", "imu", "gnss", "rmse_pos", "final_pos", "best"]
    hdr = " | ".join(hdr_cols)
    sep = " | ".join("---" for _ in hdr_cols)
    fh.write(hdr + "\n" + sep + "\n")
    for r in rows:
        fh.write(
            f"{r['dataset']} | {r['method']} | {r['imu']} | {r['gnss']} | {r['rmse_pos']:.2f} | {r['final_pos']:.2f} | {r['best']}\n"
        )

print("Created results/summary.csv and results/summary.md")
