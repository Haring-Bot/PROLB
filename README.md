# Mobile Robot Localization: KF vs EKF vs PF

[cite_start]This project implements and compares three probabilistic localization algorithms for a Turtlebot 3 in a ROS2 Jazzy environment[cite: 10, 92, 93]. [cite_start]It evaluates the trade-offs between linear/nonlinear estimation and computational resource management[cite: 11, 18, 20].

## Performance Comparison

| Metric | Kalman Filter (KF) | Extended Kalman Filter (EKF) | Particle Filter (PF/MCL) |
| :--- | :--- | :--- | :--- |
| **Accuracy** | [cite_start]High in linear paths; drifts in sharp turns[cite: 132, 133]. | [cite_start]Sufficient tracking; sensitive to linearization errors[cite: 137, 140]. | [cite_start]**Highest**; corrects drift using laser scans[cite: 145, 146, 151]. |
| **CPU Usage** | [cite_start]10.3%[cite: 161]. | [cite_start]**7.0%**[cite: 161]. | [cite_start]73.8% (at 300 particles/36 rays)[cite: 159, 161]. |
| **Noise Model** | [cite_start]Gaussian/Linear[cite: 40]. | [cite_start]Linearized Nonlinear[cite: 71]. | [cite_start]Non-Gaussian/Non-parametric[cite: 82, 83]. |



## 🛠 Tech Stack
* [cite_start]**Middleware:** ROS2 Jazzy [cite: 92]
* [cite_start]**Simulation:** Gazebo (Physics) & RVIZ2 (Visualization) [cite: 94]
* [cite_start]**Navigation:** Nav2 package [cite: 102]
* [cite_start]**Hardware Profile:** Turtlebot 3 [cite: 93]
* [cite_start]**Language:** C++ using the Eigen library [cite: 107]

## Key Implementations

### Kalman Filters (KF & EKF)
* [cite_start]Uses `cmd_vel` for the motion model and `odom`/`imu` for corrections[cite: 95, 108].
* [cite_start]The EKF linearizes nonlinear dynamics using **Jacobian matrices** calculated on-the-fly[cite: 71, 72, 116].
* [cite_start]Maintains a 6x1 state vector $\mu$ tracking position, orientation, and velocity[cite: 95, 98].

### Particle Filter (MCL)
* [cite_start]Approximates the posterior distribution using a set of weighted samples (particles)[cite: 83].
* [cite_start]Implements a **likelihood field** to compare expected laser scans with actual data[cite: 124].
* [cite_start]Uses a **resampling wheel algorithm** to maintain high-probability hypotheses[cite: 125].

## Setup & Usage

1. **Launch everything:**
   ```bash
   ros2 launch start.launch.py
