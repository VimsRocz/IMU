# Initial Attitude Determination

**Task&nbsp;3** solves Wahba's problem using the TRIAD, Davenport and SVD methods. With the measured pairs $(\mathbf{g}_b,\mathbf{g}_{\text{NED}})$ and $(\boldsymbol{\omega}_{ie,b},\boldsymbol{\omega}_{ie,\text{NED}})$ the rotation matrix $\mathbf{C}_{b}^{\text{NED}}$ is estimated.

The TRIAD algorithm forms two orthonormal bases
$$
\mathbf{b}_1 = \frac{-\mathbf{g}_b}{\|\mathbf{g}_b\|}, \qquad
\mathbf{b}_2 = \frac{\mathbf{b}_1 \times \boldsymbol{\omega}_{ie,b}}{\|\mathbf{b}_1 \times \boldsymbol{\omega}_{ie,b}\|}.
$$
Stacking these basis vectors yields $\mathbf{C}_{b}^{\text{NED}}$.

The attitude errors are evaluated as angles between the reference and estimated vectors. Typical gravity misalignment is under $0.1^\circ$ while the Earth‑rate error grows from $0.0002^\circ$ for dataset **X001** to nearly $2^\circ$ for **X003**.

Plots are written to `results/run_triad_only/<tag>_task3_errors_comparison.pdf` and `results/run_triad_only/<tag>_task3_quaternions_comparison.pdf`.
