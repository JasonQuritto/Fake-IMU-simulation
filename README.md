# Asynchronous Multi-IMU 6DoF Sequential Kalman Filter (ROS 2)

A ROS 2 package designed for fusing multiple asynchronous IMU sensor streams using a **6DoF Sequential Kalman Filter (SKF)**. The filter estimates 3D orientation (Roll, Pitch, Yaw) and angular velocities ($w_x, w_y, w_z$) while handling sensor outages, high-frequency noise, and angle wrapping discontinuities.

---

## Key Features

* **Asynchronous Sensor Fusion:** Decoupled architecture separating timer-driven predictions from callback-driven sequential measurement updates.
* **Angle Normalization (Wrapping):** Innovation vector and state updates are mapped to $[-\pi, \pi]$ using standard two-argument arctangent logic, avoiding boundary jump instabilities near $\pm\pi$.
* **Robust Outlier Rejection:** $\chi^2$ Mahalanobis distance gating thresholded for $k=6$ degrees of freedom ($d_{thresh} = 16.81$ at $p=0.99$) rejects corrupted readings.
* **Deadlock Recovery:** Features a consecutive rejection counter and covariance inflation mechanism ($\mathbf{P} \leftarrow 1.15 \times \mathbf{P}$) to prevent tracking loss cascades after temporary sensor outages.
* **Numerical Stability:** State covariance matrix update utilizes the **Joseph form** to guarantee positive semi-definiteness:
  $$\mathbf{P}_k = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_{k|k-1} (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T$$

---

## State Vector & System Model

### State Vector
$$\mathbf{x} = \begin{bmatrix} \phi & \theta & \psi & \omega_x & \omega_y & \omega_z \end{bmatrix}^T$$
* $\phi, \theta, \psi$: Roll, Pitch, Yaw angles (rad)
* $\omega_x, \omega_y, \omega_z$: Angular velocities ($\text{rad/s}$)

### Measurement Vector
$$\mathbf{z} = \begin{bmatrix} \phi_m & \theta_m & \psi_m & \omega_{x,m} & \omega_{y,m} & \omega_{z,m} \end{bmatrix}^T$$

---

## File Structure

* `SequentialKalmanFilter.py`: Core matrix mathematics, Joseph-form covariance updates, Mahalanobis gating, and angle wrapping routines.
* `Fake_IMU_Publisher.py`: Synthetic multi-IMU ROS 2 publisher node generating sinusoidal motions, Gaussian noise, and artificial packet loss.
* `Multi_IMU_Fusion_Subscriber.py`: Main ROS 2 node running timer-based `predict()` calls and callback-driven sequential `update()` calls for each IMU stream.

---

## Prerequisites & Installation

### Requirements
* **ROS 2** (Humble / Iron / Jazzy)
* **Python 3.x**
* **NumPy**

### Build
Place the package inside your ROS 2 workspace `src` directory and build:

```bash
cd ~/ros2_ws
colcon build --packages-select <your_package_name>
source install/setup.bash