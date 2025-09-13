
"""Simple Tkinter GUI wrapper around the run_*_only helpers.

The standalone scripts ``run_triad_only.py``, ``run_davenport_only.py`` and
``run_svd_only.py`` perform the full GNSS/IMU processing pipeline for a single
dataset.  This GUI offers a thin front-end that lets the user pick the input
files and choose which method to run.  The script executed mirrors the console
behaviour documented in :mod:`run_triad_only.py`, so the verbose task logs and
result paths match the command-line tools.
"""

import tkinter as tk
from tkinter import ttk, filedialog
import subprocess
import threading
import sys
import re
from pathlib import Path

# Available attitude-initialisation methods and their corresponding helper
# scripts.  These scripts print the detailed task logs seen in the CLI examples.
SCRIPT_MAP = {
    "TRIAD": "run_triad_only.py",
    "Davenport": "run_davenport_only.py",
    "SVD": "run_svd_only.py",
    "All methods": "run_all_methods_single.py",
}
METHODS = list(SCRIPT_MAP.keys())


def _infer_dataset_id(imu: str, gnss: str) -> str:
    """Infer dataset ID (e.g. ``X002``) from the selected filenames.

    If no dataset tag is found, fall back to the default used by the scripts
    (``X002``).
    """

    pattern = re.compile(r"(X\d{3})")
    for p in (imu, gnss):
        m = pattern.search(Path(p).name)
        if m:
            return m.group(1)
    return "X002"


def run_fusion(imu: str, gnss: str, truth: str, method: str, log_widget: tk.Text):
    script = SCRIPT_MAP[method]
    dataset = _infer_dataset_id(imu, gnss)
    cmd = [
        sys.executable,
        "-u",  # ensure subprocess output is unbuffered
        str(Path(__file__).resolve().parent / "PYTHON" / "src" / script),
        "--imu",
        imu,
        "--gnss",
        gnss,
        "--dataset",
        dataset,
    ]
    if truth:
        cmd += ["--truth", truth, "--allow-truth-mismatch"]
    else:
        # Ensure GUI runs do not implicitly auto-use any available truth
        cmd += ["--no-auto-truth"]

    # Use line-buffered pipes so output is forwarded to the log widget in real time
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    for line in iter(process.stdout.readline, ""):
        log_widget.insert(tk.END, line)
        log_widget.see(tk.END)
    process.stdout.close()
    ret = process.wait()
    log_widget.insert(tk.END, f"\nProcess finished with code {ret}\n")
    log_widget.see(tk.END)


def start_run(imu_var, gnss_var, truth_var, method_var, log_widget):
    imu = imu_var.get()
    gnss = gnss_var.get()
    truth = truth_var.get()
    method = method_var.get()
    log_widget.delete("1.0", tk.END)
    threading.Thread(
        target=run_fusion,
        args=(imu, gnss, truth, method, log_widget),
        daemon=True,
    ).start()


def choose_file(var, types):
    path = filedialog.askopenfilename(filetypes=types)
    if path:
        var.set(path)


root = tk.Tk()
root.title('GNSS IMU Fusion')

method_var = tk.StringVar(value=METHODS[0])
imu_var = tk.StringVar()
gnss_var = tk.StringVar()
truth_var = tk.StringVar()

frame = ttk.Frame(root, padding=10)
frame.grid(row=0, column=0, sticky='nsew')

# Method selection
ttk.Label(frame, text='Method:').grid(row=0, column=0, sticky='w')
method_menu = ttk.Combobox(frame, textvariable=method_var, values=METHODS, state='readonly')
method_menu.grid(row=0, column=1, sticky='ew')

# IMU file
ttk.Label(frame, text='IMU (.dat):').grid(row=1, column=0, sticky='w')
ttk.Entry(frame, textvariable=imu_var, width=40).grid(row=1, column=1, sticky='ew')
ttk.Button(
    frame,
    text="Browse",
    command=lambda: choose_file(imu_var, [("IMU files", "*.dat"), ("All", "*")]),
).grid(row=1, column=2)

# GNSS file
ttk.Label(frame, text='GNSS (.csv):').grid(row=2, column=0, sticky='w')
ttk.Entry(frame, textvariable=gnss_var, width=40).grid(row=2, column=1, sticky='ew')
ttk.Button(
    frame,
    text="Browse",
    command=lambda: choose_file(gnss_var, [("GNSS files", "*.csv"), ("All", "*")]),
).grid(row=2, column=2)

# Truth file (optional)
ttk.Label(frame, text='Truth file (optional):').grid(row=3, column=0, sticky='w')
ttk.Entry(frame, textvariable=truth_var, width=40).grid(row=3, column=1, sticky='ew')
ttk.Button(
    frame,
    text="Browse",
    command=lambda: choose_file(truth_var, [("All", "*")]),
).grid(row=3, column=2)

# Run button
ttk.Button(frame, text='Run', command=lambda: start_run(imu_var, gnss_var, truth_var, method_var, log_text)).grid(row=4, column=0, columnspan=3, pady=5)

# Log area
log_text = tk.Text(root, height=15)
log_text.grid(row=1, column=0, sticky='nsew')

root.columnconfigure(0, weight=1)
root.rowconfigure(1, weight=1)
root.mainloop()
