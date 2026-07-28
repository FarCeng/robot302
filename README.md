# Robot302

An autonomous warehouse inventory monitoring robot developed by Team Autonix for the ICIT 2026 Robotics Competition.

The robot autonomously navigates warehouse aisles using ROS 2 and LiDAR while performing real-time inventory monitoring through computer vision. It combines autonomous navigation, object detection, and inventory reporting into a modular robotic platform. 

---

## Overview

Traditional warehouse inventory management relies heavily on manual stock-taking, which is prone to human error and operational inefficiency. Robot302 is designed to automate this process by integrating:

- Autonomous navigation using ROS 2 Navigation2
- LiDAR-based SLAM and localization
- AI-based object detection using YOLO
- Real-time inventory monitoring dashboard

The robot can patrol warehouse aisles, stop at predefined inspection points, detect and count objects on shelves, and report inventory information automatically.

---

## Main Features

- Autonomous warehouse navigation
- 2D LiDAR SLAM
- AMCL localization
- Navigation2 waypoint navigation
- Differential drive mobile robot
- micro-ROS communication with ESP32
- YOLO object detection
- Inventory counting
- Real-time monitoring dashboard
- Modular ROS2 architecture

---

## System Architecture

```
                +----------------------+
                |   Monitoring GUI     |
                +----------+-----------+
                           |
                           |
                    Inventory Report
                           |
+------------------------------------------------------+
|                     Intel NUC                         |
|                                                      |
|  ROS 2                                               |
|  ├── Navigation2                                     |
|  ├── SLAM Toolbox                                    |
|  ├── AMCL                                             |
|  ├── YOLO Detection                                  |
|  ├── Mission Manager                                 |
|  └── Dashboard Publisher                             |
+----------------------+-------------------------------+
                       |
                  micro-ROS
                       |
+----------------------+-------------------------------+
|                      ESP32                           |
|                                                      |
| Encoder │ IMU │ Motor Driver │ PID │ Odometry       |
+----------------------+-------------------------------+
                       |
               Differential Drive Robot
```

---

## Hardware

- Intel NUC
- ESP32
- 2D LiDAR
- RGB Camera
- IMU
- DC Motors with Encoder
- Lithium-Ion Battery
- LCD Display

---

## Software Stack

| Component | Technology |
|-----------|------------|
| Robot Framework | ROS 2 Humble |
| Navigation | Navigation2 |
| Mapping | SLAM Toolbox |
| Localization | AMCL |
| Communication | micro-ROS |
| Vision | YOLO |
| Programming | C++, Python |
| Firmware | PlatformIO |
| Dashboard | Python GUI |

---

## Repository Structure

```
robot302/

├── docs/
│
├── firmware/
│   └── robot302_controller/
│
├── robot302_ws/
│   ├── src/
│   ├── build/
│   ├── install/
│   └── log/
│
├── scripts/
│   ├── start_robot302.py
│   └── start_sim.py
│
├── README.md
└── .gitignore
```

---

## Build

```bash
cd robot302/robot302_ws

colcon build

source install/setup.bash
```

---

## Run

Example:

```bash
python3 scripts/start_robot302.py
```

Simulation:

```bash
python3 scripts/start_sim.py
```

---

## Team

Team **Autonix**

- Muhammad Endrizal Rahman
- Bari Salmani
- Farrel Aryaputra Andanu
- Lugas Ade Mustara
- Surya Elha Welyan Gogo Saragi

Supervisor

Dr. Arjon Turnip, ST., MT., Ph.D.

---

## Research Focus

This project integrates several robotics research topics:

- Autonomous Mobile Robot
- Warehouse Automation
- SLAM
- Navigation2
- Computer Vision
- Object Detection
- Inventory Monitoring
- Machine Learning
- Human-Robot Collaboration

---

## Future Development

- Multi-floor navigation
- Dynamic obstacle prediction
- Automatic docking and charging
- Multi-robot coordination
- Warehouse Management System (WMS) integration
- Cloud monitoring dashboard

---

## License

This repository is currently intended for research and academic purposes.
