# YOLOv8-Guided Mobile Robot with 5DOF Arm for Autonomous Waste Sorting 🤖

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Arduino](https://img.shields.io/badge/Arduino-Mega2560-green.svg)](https://www.arduino.cc/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.5+-red.svg)](https://opencv.org/)

> 🎓 Final year project implementing an autonomous waste sorting robot using computer vision and robotics.

<p align="center">
  <img src="![IMG_6712](https://github.com/user-attachments/assets/50f6a22b-fe3a-4e8e-aa47-f365496048cd)
" alt="MegaPi Robot" width="600"/>
</p>

## 📝 Table of Contents
- [About](#about)
- [Features](#features)
- [Architecture](#architecture)
- [Getting Started](#getting-started)
- [Current Limitations](#current-limitations)
- [Roadmap](#roadmap)
- [Contributing](#contributing)
- [Authors](#authors)
- [Acknowledgments](#acknowledgments)
- [License](#license)

## 🤔 About <a name = "about"></a>

This project implements an autonomous waste sorting robot that combines computer vision, robotics, and machine learning. Using a YOLOv8n model for waste detection and ArUco markers for bin identification, the robot can identify, collect, and sort different types of waste materials.

📄 Our research paper can be found here: [Link to be added later]

### 🎯 Key Features <a name = "features"></a>

- 🤖 Autonomous waste detection and sorting
- 🧠 Custom-trained YOLOv8n model (0.941 mAP@50)
- 🦾 5-DOF robotic arm for precise manipulation
- 🎯 ArUco marker-based bin detection
- 📱 Android app for remote control
- 🎛️ Dual microcontroller architecture
- 🔄 Real-time object detection

## 🏗️ Architecture <a name = "architecture"></a>

### Hardware Stack
```
Robot/
│
├── Brain/
│   ├── Raspberry Pi 4 (8GB)
│   └── Arduino Mega 2560
│
├── Sensors/
│   ├── RPi Camera v2
│   ├── HC-SR04 Ultrasonic
│   └── HC-05 Bluetooth
│
├── Actuators/
│   ├── 3x MG996R Servos
│   ├── 3x SG90 Servos
│   └── 4x TT DC Motors
│
└── Controllers/
    ├── PCA9685 PWM Driver
    └── L293D Motor Driver
```

### Software Stack
```
Software/
│
├── Vision/
│   ├── YOLOv8n Custom Model
│   └── OpenCV (ArUco + Image Processing)
│
├── Control/
│   ├── Python High-level Control
│   └── Arduino C++ Low-level Control
│
└── Interface/
    └── Kotlin/Jetpack Compose App
```

## 🚀 Getting Started <a name = "getting-started"></a>

A final year project by Haouioui Sid Ahmed and Houhou Mohamed Dhia Eddine that implements an autonomous waste sorting robot using computer vision and robotics.

Our paper can be found here: [Link to be added]

## Project Overview

This project implements an autonomous waste sorting robot that combines computer vision, robotics, and machine learning to identify, collect, and sort different types of waste materials. The robot uses a YOLOv8n model for waste detection and ArUco markers for bin identification, all controlled through a Raspberry Pi 4 and Arduino Mega architecture.

### Key Features

- Autonomous waste detection and sorting
- Custom-trained YOLOv8n model for waste classification
- 5-DOF robotic arm for precise object manipulation
- ArUco marker-based bin detection system
- Android app for remote control and monitoring
- Dual microcontroller architecture (Raspberry Pi 4 + Arduino Mega)
- Real-time object detection and distance sensing

## System Architecture

### Hardware Components

- Raspberry Pi 4 (8GB) for high-level control and vision processing
- Arduino Mega 2560 for low-level hardware control
- Custom 3D printed chassis and robotic arm
- 6 servo motors (3x MG996R, 3x SG90) for arm control
- 4 TT DC motors for mobility
- Raspberry Pi Camera Module v2
- HC-SR04 ultrasonic sensor
- HC-05 Bluetooth module
- PCA9685 PWM driver for servo control
- L293D motor driver for DC motors

### Software Stack

- Custom YOLOv8n model for waste detection
- OpenCV for image processing and ArUco detection
- Python for high-level control
- Arduino C++ for low-level control
- Kotlin/Jetpack Compose for Android app
- Custom state machines for autonomous behavior

## Implementation Details

### Vision System
- Trained on a custom dataset from [Roboflow Universe](https://universe.roboflow.com/ai-project-i3wje/waste-detection-vqkjo/dataset/10)
- Detects three waste categories: metal, plastic, and paper
- Achieves 0.941 mAP@50 accuracy
- Real-time processing on Raspberry Pi 4

### Robotic Arm
The 5-DOF robotic arm is based on the design from [OmArTronics](https://omartronics.com/diy-6-dof-robotic-arm-with-bluetooth-control-design-build-and-program/), modified and optimized for our specific use case. It features:
- 5 degrees of freedom for complex manipulation
- Custom-designed 3D printed components
- Precision servo control through PCA9685
- Pre-programmed movement sequences for reliable operation

### Mobile Application
An Android application (to be uploaded) provides:
- Manual and autonomous mode control
- Real-time status monitoring
- Speed control in manual mode
- Bluetooth connectivity

## Demonstration

See our robot in action: [YouTube Demo](https://youtu.be/VdJLIz-9X8A)

## Future Improvements

- Integration of 3D depth sensing
- Enhanced motion control with encoder feedback
- LiDAR-based distance sensing
- ROS implementation
- Improved power management system
- Enhanced user interface with live video feed

## ⚠️ Current Limitations <a name = "current-limitations"></a>

### Robotic Arm and Manipulation
- [ ] Fixed movement patterns using pre-set positions
- [ ] Limited adaptability to object variations
- [ ] No real-time position feedback

### Locomotion and Chassis
- [ ] No encoder feedback for precise movement
- [ ] Timer-based turning (accuracy issues)
- [ ] Insufficient motor/wheel strength for full weight

### Perception and Sensing
- [ ] Unreliable ultrasonic readings
- [ ] Limited 2D vision (no depth perception)
- [ ] ArUco marker dependency for bin detection
- [ ] Limited field of view (no environment scanning)

### Performance
- [ ] Slow object detection (~1 FPS on RPi)
- [ ] Occasional misclassifications
- [ ] Basic state machine architecture
- [ ] Limited Bluetooth range/speed (HC-05)

### Power Management
- [ ] Dual power source complexity
- [ ] No battery monitoring system
- [ ] No low power handling

## 🛣️ Roadmap <a name = "roadmap"></a>

### Phase 1: Enhanced Perception
- [ ] Integrate 3D depth camera
- [ ] Implement inverse kinematics
- [ ] Add LiDAR sensor
- [ ] Improve object detection speed with Edge TPU

### Phase 2: Improved Control
- [ ] Add motor encoders
- [ ] Implement ROS architecture
- [ ] Upgrade to stronger actuators
- [ ] Add battery monitoring

### Phase 3: Software Upgrades
- [ ] Migrate to ROS2
- [ ] Implement SLAM
- [ ] Add autonomous charging
- [ ] Enhance app features


## ✍️ Authors <a name = "authors"></a>

- [@Haouioui Sid Ahmed](mailto:haouiouiahmed0@gmail.com)
- [@Houhou Mohamed Dhia Eddine](mailto:Dhayoohouhou@gmail.com)

## 🎉 Acknowledgments <a name = "acknowledgments"></a>

- Dr. Terki Nadjiba - Project Supervisor
- [OmArTronics](https://omartronics.com/diy-6-dof-robotic-arm-with-bluetooth-control-design-build-and-program/) - Robotic Arm Design
- [Roboflow Universe](https://universe.roboflow.com/ai-project-i3wje/waste-detection-vqkjo/dataset/10) - Dataset

## 📝 License <a name = "license"></a>

This project is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

### Terms
- ✅ Share and Adapt
- ❌ No Commercial Use
- 📝 Must Give Credit
- 🔄 Share Under Same License

For more details, see the [full license text](https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode).

### Citation
```bibtex
@misc{yolov8_waste_sorting_robot,
  title={YOLOv8-Guided Mobile Robot with 5DOF Arm for Autonomous Waste Sorting},
  author={Haouioui, Sid Ahmed and Houhou, Mohamed Dhia Eddine},
  year={2024},
  publisher={GitHub},
  howpublished={\url{https://github.com/SidCodesRobotics/yolo-robot-picknplace}}
}
``` 
