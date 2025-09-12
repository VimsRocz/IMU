
# GUI for GNSS/IMU fusion
import tkinter as tk
from tkinter import ttk, filedialog
import subprocess, threading, sys
from pathlib import Path

METHODS = ['TRIAD','Davenport','SVD']

def run_fusion(imu, gnss, truth, method, log_widget):
    cmd = [
        sys.executable,
        str(Path(__file__).resolve().parent / 'PYTHON' / 'src' / 'GNSS_IMU_Fusion.py'),
        '--imu-file', imu,
        '--gnss-file', gnss,
        '--method', method,
    ]
    if truth:
        cmd += ['--truth-file', truth, '--allow-truth-mismatch']
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    for line in process.stdout:
        log_widget.insert(tk.END, line)
        log_widget.see(tk.END)
    process.wait()

def start_run(imu_var, gnss_var, truth_var, method_var, log_widget):
    imu = imu_var.get(); gnss = gnss_var.get(); truth = truth_var.get(); method = method_var.get()
    log_widget.delete('1.0', tk.END)
    threading.Thread(target=run_fusion, args=(imu, gnss, truth, method, log_widget), daemon=True).start()

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
ttk.Button(frame, text='Browse', command=lambda: choose_file(imu_var, [('IMU files','*.dat'),('All','*')])).grid(row=1, column=2)

# GNSS file
ttk.Label(frame, text='GNSS (.csv):').grid(row=2, column=0, sticky='w')
ttk.Entry(frame, textvariable=gnss_var, width=40).grid(row=2, column=1, sticky='ew')
ttk.Button(frame, text='Browse', command=lambda: choose_file(gnss_var, [('GNSS files','*.csv'), ('All','*')])).grid(row=2, column=2)

# Truth file
ttk.Label(frame, text='Truth file:').grid(row=3, column=0, sticky='w')
ttk.Entry(frame, textvariable=truth_var, width=40).grid(row=3, column=1, sticky='ew')
ttk.Button(frame, text='Browse', command=lambda: choose_file(truth_var, [('All','*')])).grid(row=3, column=2)

# Run button
ttk.Button(frame, text='Run', command=lambda: start_run(imu_var, gnss_var, truth_var, method_var, log_text)).grid(row=4, column=0, columnspan=3, pady=5)

# Log area
log_text = tk.Text(root, height=15)
log_text.grid(row=1, column=0, sticky='nsew')

root.columnconfigure(0, weight=1)
root.rowconfigure(1, weight=1)
root.mainloop()
