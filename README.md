# Introduction to Robotics (2025)
## Sookmyung W. Univ.
## Lecturer: Prof. Joo Yong Sim
## Tutorial 1: Two-Link Manipulator PD Control — Tutorial

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
