# Sim-to-Real Transfer for the Pepper Robot

This repository contains all the code, simulation assets, and training environments used in my research on **Sim-to-Real Transfer for the Pepper Robot**.
The project explores how reinforcement learning policies trained in simulation can be effectively transferred to a real Pepper robot for autonomous navigation and interaction tasks.

---

## 📘 Overview

The goal of this research is to bridge the gap between **simulation and reality** by:
1. Capturing a **photogrammetric 3D model** of a real indoor environment.
2. Integrating that model into **Gazebo** for simulation.
3. Training **reinforcement learning (RL)** policies (using PPO) within this high-fidelity environment.
4. Deploying the learned policy on a **real Pepper robot** and comparing performance metrics.

This project evaluates how closely the simulated and real-world behaviors align, contributing to improved transferability and navigation reliability in social robots.

---

## 🧠 Key Components

### 🏠 Photogrammetry
- High-resolution 3D model of a real environment captured using **Polycam**.
- Post-processed and optimized in **Blender**.
- Exported for integration into Gazebo with custom invisible floors and lighting.

### 🤖 Simulation Environment
- Custom **Pepper robot URDF** and **ROS Noetic** integration.
- Gazebo-based environment with realistic physics and sensors (camera, laser, IMU).
- ROS topics for `/pepper/cmd_vel`, `/pepper/camera/front/image_raw`, and others.

### 🎮 Reinforcement Learning
- RL training implemented with **Stable-Baselines3 (PPO)**.
- Custom **Gym environment** (`PepperEnv`) that interfaces with ROS and Gazebo.
- Reward functions based on target proximity, collision avoidance, and smooth navigation.

### 🧍 Real-World Testing
- Deployment scripts for executing learned policies on the **real Pepper robot**.
- Data collection via ROS topics and trajectory logging.

---

