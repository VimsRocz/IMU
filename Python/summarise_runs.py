#!/usr/bin/env python3
"""
Parse logs/* for lines that start with [SUMMARY] and emit:
  summary.csv
  summary.md
"""

import csv, pathlib, re, os

RESULTS_DIR = pathlib.Path(os.environ.get("IMU_OUTPUT_DIR", "results"))
RESULTS_DIR.mkdir(exist_ok=True)

LOG_DIR = pathlib.Path("logs")
SUMMARY = re.compile(r"\[SUMMARY\]\s+(.*)")

rows = []
for log in LOG_DIR.glob("*.log"):
    for line in log.read_text().splitlines():
        m = SUMMARY.search(line)
        if m:
            kv = dict(pair.split("=", 1) for pair in m.group(1).split())
            rows.append(kv)

# CSV -------------------------------------------------------------------------
with open(RESULTS_DIR / "summary.csv", "w", newline="") as fh:
    writer = csv.DictWriter(
        fh, fieldnames=["method", "imu", "gnss", "rmse_pos", "final_pos"]
    )
    writer.writeheader()
    writer.writerows(rows)

# Markdown table --------------------------------------------------------------
with open(RESULTS_DIR / "summary.md", "w") as fh:
    hdr = " | ".join(["method", "imu", "gnss", "rmse_pos", "final_pos"])
    sep = " | ".join("---" for _ in range(5))
    fh.write(hdr + "\n" + sep + "\n")
    for r in rows:
        fh.write(" | ".join(r.values()) + "\n")

print(f"Created {RESULTS_DIR}/summary.csv and {RESULTS_DIR}/summary.md")
