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
4. Images are analyzed using a machine learning model to determine plant health
5. Farmers can trace affected regions using GPS data for targeted action

---

## 🎯 Future Goals
- Implement **GPS–Camera fusion** for accurate localization of detected plant issues
- Enable **autonomous navigation** using ROS 2 navigation stack
- Integrate **machine learning models** for plant disease and pest detection
- Develop a **real-time monitoring dashboard**
- Improve **terrain adaptability** and gait control algorithms

---

## 🧩 Key Features
- 12-DOF quadruped design (3 DOF per leg)
- ROS 2 and micro-ROS integration
- Modular and lightweight mechanical design
- GPS and IMU-based sensor fusion for navigation and stability
- Expandable for future ML and vision-based tasks

---

## 🛠️ Current Progress
- ✅ Mechanical design finalized using aluminium extrusions, acrylic, and 3D printed parts
- ✅ Hardware integration of servo motors, drivers, and sensors
- ⚙️ Working on ROS 2 and micro-ROS communication setup
- 🔜 Next step: GPS–Camera data fusion and machine learning model integration

---

## 📸 Media
CAD Model
<img width="1274" height="1137" alt="17613221658659206866258960243069" src="https://github.com/user-attachments/assets/f5eb925e-0b16-4f58-9641-ccb9edd77e6b" />



## 🧾 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
