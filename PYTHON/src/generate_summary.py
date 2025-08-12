"""Assemble a PDF report summarising all fusion runs.

Usage:
    python generate_summary.py

The script reads ``results/summary.csv`` produced by ``summarise_runs.py`` and
creates ``results/project_summary.pdf`` with one page per dataset. Each page
lists the RMSE and final error metrics and embeds the corresponding plot PDF if
available. This utility mirrors the MATLAB helper ``validate_pipeline_summary.m``
for cross-language comparison.
"""

import os
import logging
import pandas as pd
from fpdf import FPDF

logging.basicConfig(level=logging.INFO, format="%(message)s")

# Load run results produced by ``summarise_runs.py``
DF_PATH = "results/summary.csv"


def main():
    os.makedirs('results', exist_ok=True)
    logging.info("Ensured 'results/' directory exists.")
    df = pd.read_csv(DF_PATH)

    pdf = FPDF()
    pdf.set_auto_page_break(True, margin=15)
    for _, row in df.iterrows():
        pdf.add_page()
        pdf.set_font("Arial", "B", 16)
        ds = f"{row['imu']} / {row['gnss']}"
        pdf.cell(0, 10, f"{ds} - {row['method']}", ln=True)
        pdf.set_font("Arial", size=12)
        pdf.multi_cell(0, 8, f"RMSEpos = {row['rmse_pos']} m, Final Error = {row['final_pos']} m")
        base = os.path.splitext(row['imu'])[0]
        image = f"results/{base}_{row['method']}_plots.pdf"
        if os.path.exists(image):
            pdf.image(image, w=180)

    pdf.output('results/project_summary.pdf')


if __name__ == '__main__':
    main()
