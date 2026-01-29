# 🚁 ROS 2 Drone State Estimation

![ROS 2](https://img.shields.io/badge/ROS2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![MATLAB](https://img.shields.io/badge/MATLAB-R2024a-FF69B4?style=for-the-badge&logo=matlab&logoColor=white)
![License](https://img.shields.io/badge/License-Apache%202.0-D22128?style=for-the-badge)

> **Advanced State Estimation & Control Simulation using Extended Kalman Filters.**

This project implements a modular ROS 2 workspace designed for simulating and estimating the state of a drone. It features a custom controller, a service-based architecture for computations, and a robust **Extended Kalman Filter (EKF)** for fusing Odometry and IMU data.

---



## 🧮 MATLAB Prototyping

To validate the algorithms before deployment, we developed prototype implementations of **Standard Kalman Filters** in MATLAB. These scripts demonstrate the core logic used for state estimation.
*(See graphs below in the Performance Reports section)*



- **`FK1.m`**: **Basic 2D Position Tracking**

  - Uses a simple state vector `[x, y]`.
  - **Process Model**: Constant position with random walk.
  - **Observation**: Direct noisy position measurements.

  <p align="center">
    <img src="results/kalman_trajectory1.svg" alt="FK1" width="600"/>
  </p>

- **`FK2.m`**: **Constant Velocity Model with Signal Loss Simulation**
  - Expanded state vector `[x, y, vx, vy]`.
  - **Process Model**: Constant velocity kinematics.
  - **Observation**: Position only.
  - **Feature**: Simulates GPS signal loss (missing data) and relies on prediction steps during outages.

  <p align="center">
    <img src="results/kalman_trajectory2.svg" alt="FK2" width="600"/>
  </p>

- **`FK3.m`**: **Advanced Velocity Inference**
  - Uses the same Constant Velocity Model as `FK2`.
  - **Feature**: Observation matrix includes previous states to better infer velocity from position changes.
  - Also handles missing measurement data.

  <p align="center">
    <img src="results/kalman_trajectory3.svg" alt="FK3" width="600"/>
  </p>

---

## ⚡ Standard Kalman Filter Performance Reports (Practice 2)

### 🎯 Objectives

- Understand and implement a basic Kalman Filter to estimate robot position.
- Extend the state model to include linear and angular velocity.
- Study filter behavior under different noise configurations (process and measurement).
- Prepare the foundation for implementing an Extended Kalman Filter (EKF) in future practices.

### 🏗️ Implementation Structure

The ROS 2 implementation is organized as follows:

- **`kalman_filter.py`**: Contains the `KalmanFilter` and `KalmanFilter_2` classes.
- **`motion_models.py`**: Defines the linear motion model.
- **`observation_models.py`**: Defines the observation model.
- **`kf_estimation.py`**: Main ROS 2 node that executes the filter.
- **`sensor_utils.py`**: Utility functions for data extraction and normalization.
- **`visualization.py`**: Manages estimation visualization.

### 📊 Filter Models

#### Model 1 – Basic Kalman Filter (`KalmanFilter`)

- **State Vector**: 2D position `(x, y, θ)`
- **Observation**: Position measured with noise
- **Control Input**: Linear and angular velocity
- Implements a linear motion model and direct position observation

#### Model 2 – Extended State Kalman Filter (`KalmanFilter_2`)

- **State Vector**: Extended `(x, y, θ, vx, vy, ω)`
- **Observation**: Position and velocities with noise
- **Control Input**: Linear velocity in x, y and rotational
- Better incorporates real robot motion dynamics

### 🧪 Experimental Results

We conducted experiments with three noise configurations, generating the corresponding performance graphs.

#### 🔵 Case 1 – Low Noise (Default Configuration)
*Low values for initial covariance matrix `Q` and measurement noise `R`.*

<p align="center">
  <img src="results/kf_posicion_sinruido.png" alt="Position - Low Noise" width="400"/>
  <img src="results/kf_vel_sinruido.png" alt="Velocity - Low Noise" width="400"/>
</p>
| The filter accurately tracks the robot trajectory. Estimation is very close to the actual path. | The model trusts both the process and the measurement equally. |

#### 🔴 Case 2 – High Measurement Noise
*Low values for `Q` matrix. Measurement noise `R` multiplied by 5: `noise_std = np.array([0.02, 0.02, 0.01, 0.02, 0.02, 0.01]) * 5`*

<p align="center">
  <img src="results/kf_posicion_ruidoaltomed.png" alt="Position - High Measurement Noise" width="400"/>
  <img src="results/kf_vel_ruidoaltomedida.png" alt="Velocity - High Measurement Noise" width="400"/>
</p>
| The filter has significant error because measurements are highly inaccurate. The estimated trajectory is very erratic. | Good compensation is observed thanks to the motion model. |

#### 🟠 Case 3 – High Process Noise
*Low values for measurement noise `R`. Initial covariance `Q` multiplied by 100: `initial_covariance = np.eye(3) * 100`*

<p align="center">
  <img src="results/kf_posicion_ruidoaltoproceso.png" alt="Position - High Process Noise" width="400"/>
  <img src="results/kf_vel_ruidoaltoproceso.png" alt="Velocity - High Process Noise" width="400"/>
</p>
| The filter reacts less abruptly to noise. For the pure Kalman Filter, a small offset separates the estimated trajectory from the real one. | This demonstrates how `Q` (process noise) directly affects trust in the dynamic model. |

### 📝 Technical Analysis

- **Q Matrix (Process Noise)**: Controls how much the filter trusts its prediction vs. the measurement.
  - High `Q` implies more uncertainty in the model, useful for unpredictable robot movements.
  
- **R Matrix (Measurement Noise)**: Allows mitigation of very noisy measurements.
  - Properly tuning `R` helps filter out sensor errors.

- **Extended Model**: The velocity-enhanced model provides richer and more realistic estimation, though it's also more sensitive to velocity observation errors.

---


## 📈 ROS 2 EKF Implementation Results

Following the standard KF validation, we implemented **Extended Kalman Filters (EKF)** within the ROS 2 workspace. We tested 7D and 8D state vectors on circular trajectories.

<p align="center">

| 7D EKF Circle Test | 8D EKF Circle Test |
|:---:|:---:|
| <img src="results/7d_circle.png" alt="7D EKF" width="500"/> | <img src="results/8d_circle.png" alt="8D EKF" width="500"/> |
| **State Vector**: `[x, y, z, vx, vy, vz, yaw]` | **State Vector**: `[x, y, z, vx, vy, vz, yaw, yaw_rate]` |
| Shows good tracking but may lag in dynamic curves. | Includes `yaw_rate` for superior prediction in curved trajectories. |

</p>

---

## 📂 Project Structure

The workspace is organized into three core packages, consolidated for clarity and modularity:


- **🎮 `drone_controller`**: Directs drone navigation and acts as the driver interface.
- **🛠️ `drone_services`**: Provides computational services to offload heavy calculations.
- **🧠 `ekf_estimator`**: The brain of the operation. Implements EKF with **3D, 7D, and 8D** state vectors.

---

<p align="center">
  <i>Developed for the Advanced Robotics Course of the University of Seville</i>
</p>
