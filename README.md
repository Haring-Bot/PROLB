# Mobile Robot Localization: KF vs EKF vs PF

This project implements and compares three probabilistic localization algorithms for a Turtlebot 3 in a ROS2 Jazzy environment. It evaluates the trade-offs between linear/nonlinear estimation and computational resource management.

## Performance Comparison

| Metric | Kalman Filter (KF) | Extended Kalman Filter (EKF) | Particle Filter (PF/MCL) |
| :--- | :--- | :--- | :--- |
| **Accuracy** | High in linear paths; drifts in sharp turns. | Sufficient tracking; sensitive to linearization errors. | Highest; corrects drift using laser scans. |
| **CPU Usage** | 10.3%. | 7.0%. | 73.8% (at 300 particles/36 rays). |
| **Noise Model** | Gaussian/Linear. | Linearized Nonlinear. | Non-Gaussian/Non-parametric. |

## Tech Stack
* **Middleware:** ROS2 Jazzy
* **Simulation:** Gazebo and RVIZ2
* **Navigation:** Nav2 package
* **Hardware Profile:** Turtlebot 3
* **Language:** C++ using the Eigen library

## Key Implementations

### Kalman Filters (KF & EKF)
* Uses cmd_vel for the motion model and odom/imu for corrections.
* The EKF linearizes nonlinear dynamics using Jacobian matrices calculated on-the-fly.
* Maintains a 6x1 state vector tracking position, orientation, and velocity.

### Particle Filter (MCL)
* Approximates the posterior distribution using a set of weighted samples (particles).
* Implements a likelihood field to compare expected laser scans with actual data.
* Uses a resampling wheel algorithm to maintain high-probability hypotheses.

## Setup and Usage

1. **Launch everything:**
   ```bash
   ros2 launch start.launch.py
