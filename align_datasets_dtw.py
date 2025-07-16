import numpy as np
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean

# Update these paths for your own data
EST_FILE = "results/IMU_X002_GNSS_X002_TRIAD_kf_output.npz"
TRUTH_FILE = "STATE_X001.txt"

# Step 1: Load the data
est_data = np.load(EST_FILE)
truth_data = np.loadtxt(TRUTH_FILE, usecols=[1, 2, 3])

# Step 2: Extract position arrays
est_pos = est_data["position"]
truth_pos = truth_data

# Step 3: Normalise positions to [0, 1]
est_norm = (est_pos - est_pos.min(axis=0)) / (est_pos.max(axis=0) - est_pos.min(axis=0))
truth_norm = (truth_pos - truth_pos.min(axis=0)) / (truth_pos.max(axis=0) - truth_pos.min(axis=0))

# Step 4: Align using DTW
_, path = fastdtw(est_norm, truth_norm, dist=euclidean)
est_idx, truth_idx = zip(*path)

# Step 5: Reindex datasets along the DTW path
aligned_est = est_pos[list(est_idx)]
aligned_truth = truth_pos[list(truth_idx)]

# Step 6: Compute position errors
errors = np.linalg.norm(aligned_est - aligned_truth, axis=1)

# Optional: assign a relative time scale
relative_time = np.arange(len(aligned_est))

print(f"Aligned time range: 0 to {relative_time[-1]} units")
print(f"Position errors: mean={errors.mean():.4f}, std={errors.std():.4f}")

np.savetxt("aligned_position_errors.txt", errors)
