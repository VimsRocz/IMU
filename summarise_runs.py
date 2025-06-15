#!/usr/bin/env python3
"""
Parse logs/* for lines that start with [SUMMARY] and emit:
  summary.csv
  summary.md
"""

import csv, pathlib, re

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
with open("summary.csv", "w", newline="") as fh:
    writer = csv.DictWriter(
        fh, fieldnames=["method", "imu", "gnss", "rmse_pos", "final_pos"]
    )
    writer.writeheader()
    writer.writerows(rows)

# Markdown table --------------------------------------------------------------
with open("summary.md", "w") as fh:
    hdr = " | ".join(["method", "imu", "gnss", "rmse_pos", "final_pos"])
    sep = " | ".join("---" for _ in range(5))
    fh.write(hdr + "\n" + sep + "\n")
    for r in rows:
        fh.write(" | ".join(r.values()) + "\n")

print("Created summary.csv and summary.md")

if rows:
    widths = [8, 10, 10, 10, 10]
    header = ["method", "imu", "gnss", "RMSEpos", "End-Error"]
    print("\n" + " ".join(h.ljust(w) for h, w in zip(header, widths)))
    print(" ".join("-" * w for w in widths))
    for r in rows:
        vals = [r.get("method"), r.get("imu"), r.get("gnss"), r.get("rmse_pos"), r.get("final_pos")]
        print(" ".join(v.ljust(w) for v, w in zip(vals, widths)))

