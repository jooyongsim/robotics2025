# Introduction to Robotics (2025)
## Sookmyung W. Univ.
## Lecturer: Prof. Joo Yong Sim
## Tutorial 1: Two-Link Manipulator PD Control — Tutorial

# ✅ **TUTORIAL: 🦾 Two-Link Manipulator PD Control — Full Tutorial **

### *Robotics Lecture • Kinematics • Trajectory Generation • PD Control • Simulation*

This notebook walks through the complete process of controlling a **2-link planar manipulator** with a **joint-space PD controller**.
You will implement forward/inverse kinematics, trajectory generation, system dynamics, and closed-loop control step by step.

---

## 1. Robot & Simulation Parameters

We define the physical parameters of a 2-link planar robot arm:

* Link lengths
* Joint inertias
* Joint viscous friction
* Simulation time & step size

This forms the base model for joint-level dynamics.

Helpful reference image:
![2-link robot diagram](https://upload.wikimedia.org/wikipedia/commons/thumb/5/5a/Planar_2R_robot.svg/640px-Planar_2R_robot.svg.png)

---

## 2. Forward Kinematics (FK)

Forward kinematics provides the end-effector position ((x, y)) from joint angles (\theta_1, \theta_2):

[
x = l_1\cos\theta_1 + l_2\cos(\theta_1 + \theta_2)
]
[
y = l_1\sin\theta_1 + l_2\sin(\theta_1 + \theta_2)
]

Used later to visualize tracking in workspace.

Helpful conceptual figure:
![FK geometry](https://upload.wikimedia.org/wikipedia/commons/thumb/1/15/Planar_2R_Robot_Kinematics.svg/640px-Planar_2R_Robot_Kinematics.svg.png)

---

## 3. Inverse Kinematics (IK)

Given a desired end-effector point ((x, y)), IK computes (\theta_1) and (\theta_2).
Analytical IK is used here with two possible arm configurations:

* **Elbow-down**
* **Elbow-up**

IK geometry example:
![IK geometry](https://upload.wikimedia.org/wikipedia/commons/thumb/9/9e/Planar_two_link_robot_in_ik.png/640px-Planar_two_link_robot_in_ik.png)

---

## 4. End-Effector Target Trajectory

We generate a **smooth straight-line trajectory** between a start and goal point in workspace.

A **3-2-1 polynomial** smooths interpolation:

[
s(t) = 3s^2 - 2s^3
]

This provides continuous velocity and zero boundary acceleration.

---

## 5. Desired Joint Trajectory (θd, θ̇d)

Using IK, the workspace trajectory is transformed into:

* Desired joint angles (\theta_d(t))
* Desired joint velocities (\dot{\theta}_d(t)), computed via numerical differentiation

This desired trajectory becomes the reference for the PD controller.

---

## 6. Plant Dynamics (Joint-level Second Order System)

Each joint is modeled independently using:

[
J\ddot{\theta} + b\dot{\theta} = \tau
]

Numerical integration uses **semi-implicit Euler**, which is stable for stiff systems.

---

## 7. Joint-Space PD Controller

The joint control law:

[
\tau = K_p(\theta_d - \theta) + K_d(\dot{\theta}_d - \dot{\theta})
]

You can tune (K_p) and (K_d) to adjust tracking quality, damping, rise time, and overshoot.

---

## 8. Closed-Loop Simulation

For each time step:

1. Compute control torque
2. Integrate joint dynamics
3. Log joint states and workspace position
4. Compare against desired trajectory

---

## 9. Visualization

Plots included:

### ✔ End-effector trajectory

Desired vs actual workspace paths.

### ✔ Joint angle responses

(\theta_1(t)) and (\theta_2(t)) tracking performance.

### ✔ Joint-space tracking errors

Error signals (e_1, e_2).

Example conceptual outcome:
![PD example tracking](https://upload.wikimedia.org/wikipedia/commons/thumb/d/d0/PD_control_example.png/640px-PD_control_example.png)

---

## 🔧 Additional Experiments (Recommended)

* Tune PD gains and observe overshoot/settling behavior
* Switch IK to elbow-up mode
* Try circular and sinusoidal paths
* Add gravity terms to dynamics
* Compare joint-space vs workspace control

---

---

# 📄 **README.md VERSION (for GitHub Repository)**

```md
# Two-Link Manipulator Control Tutorial
Robotics Lecture • PD Control • Trajectory Tracking • Simulation

This repository contains a full tutorial for implementing **joint-space PD control** of a **2-link planar robotic arm**, including forward/inverse kinematics, trajectory generation, plant dynamics, and closed-loop simulation.

The main notebook is:

```

1_two_link_control.ipynb

```

---

## 🦾 Overview

This project demonstrates how to:

- Compute forward and inverse kinematics for a 2-link robot arm  
- Generate smooth end-effector trajectories  
- Convert workspace trajectories to joint trajectories  
- Implement a PD controller in joint space  
- Simulate system dynamics  
- Visualize tracking performance  

The robot model is a classic 2R planar manipulator:

![2R robot](https://upload.wikimedia.org/wikipedia/commons/thumb/5/5a/Planar_2R_robot.svg/640px-Planar_2R_robot.svg.png)

---

## 📌 Features Implemented

### ✔ Forward Kinematics  
Computes end-effector (x, y) from joint angles.

### ✔ Inverse Kinematics  
Analytical IK with elbow-up and elbow-down options.

![IK geometry](https://upload.wikimedia.org/wikipedia/commons/thumb/9/9e/Planar_two_link_robot_in_ik.png/640px-Planar_two_link_robot_in_ik.png)

### ✔ Trajectory Generation  
Smooth straight-line trajectory using a 3-2-1 polynomial for blending.

### ✔ Joint Trajectory Extraction  
Desired joint angles and velocities computed throughout the motion.

### ✔ Robot Dynamics  
Each joint modeled as a second-order system:

\[
J\ddot{\theta} + b\dot{\theta} = \tau
\]

### ✔ PD Control  
Joint-space proportional–derivative controller.

### ✔ Simulation and Visualization  
- End-effector path comparison  
- Joint angle tracking plots  
- Error analysis  

Example visualization concept:

![PD tracking](https://upload.wikimedia.org/wikipedia/commons/thumb/d/d0/PD_control_example.png/640px-PD_control_example.png)

---

## 📂 Repository Structure

```

/
├── 1_two_link_control.ipynb   # Main tutorial notebook
├── README.md                  # This document
└── (optional) images/         # Extra images if needed

```

---

## 🚀 Getting Started

Open the notebook in Google Colab:

1. Upload the `.ipynb` file  
2. Run all cells sequentially  
3. Inspect the plots and modify controller gains / trajectories  

---

## 🧪 Suggested Experiments

- Tune PD gains to see effects on stability  
- Try different trajectories (circles, curves)  
- Enable elbow-up IK  
- Add gravity terms and compare tracking  
- Replace PD with computed torque control (advanced)

---

## 📘 License

All code is free to use for educational and research purposes.  
Images included via Wikimedia Commons under their respective open licenses.

---

If you'd like, I can also generate:

✅ A version with LaTeX-rich explanations  
✅ A PDF-formatted lecture handout  
✅ A second notebook for computed-torque control (inverse dynamics)  
```

---

If you'd like adjustments (more images, more math derivations, shorter README, or a more academic tone), just tell me!
