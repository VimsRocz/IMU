# Reference and Body Vectors

This section documents **Task&nbsp;1** and **Task&nbsp;2** of the pipeline.

## Task 1 – Reference Vectors

From the first GNSS fix the initial latitude $\varphi$ and longitude $\lambda$ are computed. Gravity in the NED frame is
$$
\mathbf{g}_{\text{NED}} = \begin{bmatrix}0 \\ 0 \\ g\end{bmatrix}, \quad g \approx 9.81\,\mathrm{m/s^2}.
$$
The Earth rotation rate is
$$
\boldsymbol{\omega}_{ie,\text{NED}} = \omega_E\begin{bmatrix}\cos\varphi \\ 0 \\ -\sin\varphi\end{bmatrix},
$$
where $\omega_E=7.2921159\times10^{-5}\,\mathrm{rad/s}$.
The map showing the start location is saved as `results/<tag>_location_map.pdf`.

## Task 2 – Body-Frame Measurements

A nearly static interval is detected in the IMU data to estimate the body-frame vectors:
$$
\mathbf{g}_b = \begin{bmatrix}g_x\\g_y\\g_z\end{bmatrix},\qquad
\boldsymbol{\omega}_{ie,b} = \begin{bmatrix}\omega_x\\\omega_y\\\omega_z\end{bmatrix}.
$$
These correspond to the reference vectors expressed in the sensor axes. The estimated sampling period is roughly $2.5\,\mathrm{ms}$.
