import os
import pandas as pd
from fpdf import FPDF

# Load run results
DF_PATH = 'results/summary.csv'


def main():
    df = pd.read_csv(DF_PATH)

    pdf = FPDF()
    pdf.set_auto_page_break(True, margin=15)
    for _, row in df.iterrows():
        pdf.add_page()
        pdf.set_font('Arial', 'B', 16)
        pdf.cell(0, 10, f"Dataset {row.Dataset} - {row.Method}", ln=True)
        pdf.set_font('Arial', size=12)
        pdf.multi_cell(0, 8,
                       f"RMSEpos = {row.RMSEpos_m} m, End-Error = {row.EndErr_m} m, Runtime = {row.Runtime_s}s")
        image = f"results/{row.Dataset}_{row.Method}_plots.pdf"
        if os.path.exists(image):
            pdf.image(image, w=180)

    pdf.output('results/project_summary.pdf')


if __name__ == '__main__':
    main()
