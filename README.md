# Quadrupedal-Robot

# 🐾 Quadruped Robot using ROS 2 & micro-ROS

## 📘 Overview
This project involves the development of a **quadruped robot** with **3 Degrees of Freedom (DOF)** per leg, driven by **12 high-torque servo motors**. The robot integrates **ROS 2** (on Raspberry Pi) and **micro-ROS** (on ESP32) for distributed communication and control. The aim is to design a modular and semi-autonomous robotic platform capable of operating in agricultural environments for field inspection and plant health monitoring.

---

## ⚙️ Hardware Components
- **Actuators:** 12 × 60 kg servo motors (3 DOF per leg)
- **Controllers:**
  - Raspberry Pi (main computer running ROS 2)
  - ESP32 (running micro-ROS for motor control)
- **Sensors:**
  - IMU (for stability and orientation control)
  - GPS module (for odometry and navigation)
  - Camera module (for image capture and analysis)
- **Power System:** LiPo battery with voltage regulators
- **Structure:** Aluminium extrusion bars, acrylic sheets, and 3D-printed joints
- **Servo Driver:** 16-channel servo motor driver board

---

## 🧠 Software Architecture
- **Main Framework:** ROS 2 (Robot Operating System)
- **Communication:** micro-ROS between Raspberry Pi and ESP32
- **Stability Control:** IMU data integration for balance and orientation feedback
- **Odometry & Navigation:** GPS-based localization and waypoint following
- **Computer Vision:** Integration of camera module for real-time image capture
- **Machine Learning (Future Scope):**
  - Plant health classification using captured images
  - GPS–Camera data fusion for geotagging plant health information

---

## 🚜 Agricultural Application
This quadruped robot is designed for **agricultural field inspection and monitoring**.  
The workflow includes:
1. Robot traverses crop fields using waypoints (autonomous/semi-autonomous navigation)
2. Captures images of plants using the onboard camera
3. Fuses GPS data to map the exact location of captured images
4. Images are analyzed using a machine le
