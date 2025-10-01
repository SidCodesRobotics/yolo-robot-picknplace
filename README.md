# YOLOv8-Guided Mobile Robot with 5DOF Arm for Autonomous Waste Sorting 🤖

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Arduino](https://img.shields.io/badge/Arduino-Mega2560-green.svg)](https://www.arduino.cc/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.5+-red.svg)](https://opencv.org/)

> 🎓 Final year project implementing an autonomous waste sorting robot using computer vision and robotics.

<p align="center">
<img src="https://github.com/user-attachments/assets/50f6a22b-fe3a-4e8e-aa47-f365496048cd" alt="MegaPi" width="400"/>
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
    └──  Android App
```

# Autonomous Waste Sorting Robot

## 🚀 Getting Started <a name="getting-started"></a>

This is a final year project by Haouioui Sid Ahmed and Houhou Mohamed Dhia Eddine, focused on implementing an autonomous waste sorting robot using computer vision and robotics.

Our paper can be found here: [Link to be added]

---

### Hardware Requirements

Before you begin, ensure you have the following main **hardware components**:

* **Raspberry Pi 4 Model B** (8 GB RAM recommended): For computer vision and high-level control.
* **Arduino Mega 2560**: For motor control and low-level actuation tasks.
* **Robotic Arm**: A 5-DOF arm using SG90 and MG996R servo motors.
* **4-Wheeled Mobile Platform**: Driven by TT gear motors.
* **Raspberry Pi Camera Module v2**: Provides visual input for waste detection.
* **HC-SR04 Ultrasonic Sensor**: For distance measurement.
* **HC-05 Bluetooth Module**: For potential remote control and debugging.
* **Power Supply**:
    * A 7.4V 3000mAh LiPo battery pack for the Raspberry Pi.
    * Three 3.7V 2800mAh 18650 batteries for the Arduino and its components.
* **Motor Driver**: L293D Motor Driver Module for DC motors.
* **PWM Driver Board**: PCA9685 for controlling servo motors.
* **UART Connection**: Ensure proper wiring between the **Raspberry Pi (Serial /dev/ttyAMA0)** and **Arduino (Serial3)** for communication.

---

### Software Setup

#### 1. Arduino Code (`arduino_code.txt`)

This code controls the robot's physical movements, arm actuation, and communication with the Raspberry Pi via UART.

**Development Environment**:
Install the **Arduino IDE**.

**Libraries**:
You will need to install the following libraries within the Arduino IDE:
* `Wire.h`: For I2C communication.
* `Adafruit_PWMServoDriver.h`: For controlling the PCA9685 servo driver.
* `AFMotor.h`: For controlling the DC motors via the Adafruit Motor Shield (or L293D-based shield).

**Uploading the Code**:
1.  Open `arduino_code.txt` in the Arduino IDE.
2.  Select "**Arduino Mega 2560**" from `Tools` > `Board`.
3.  Select the correct serial port for your Arduino from `Tools` > `Port`.
4.  Upload the sketch to your Arduino Mega 2560.

#### 2. Raspberry Pi Code (`rpi_code.py`)

This Python script handles computer vision using YOLOv8n, object detection, bin identification, and sends commands to the Arduino.

**Operating System**:
Ensure your Raspberry Pi is running a compatible Linux OS (e.g., **Raspberry Pi OS**).

**Python Environment**:
The code is written in **Python 3**. Make sure Python 3 is installed.

**Libraries**:
Install the necessary Python libraries using pip:

```bash
pip install opencv-python time numpy pyserial imutils picamera2 ultralytics
```

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
An Android application (MegaPi APP) provides:
- Manual and autonomous mode control
- Real-time status monitoring
- Speed control in manual mode
- Bluetooth connectivity

## Demonstration

See our robot in action: [YouTube Demo](https://youtu.be/VdJLIz-9X8A)


## ⚠️ Current Limitations <a name = "current-limitations"></a>

### Robotic Arm and Manipulation
- [ ] Fixed movement patterns using pre-set positions
- [ ] Limited adaptability to object variations
- [ ] No real-time position feedback

### Locomotion and Chassis
- [ ] No encoder feedback for precise movement
- [ ] Timer-based turning (accuracy issues)

### Perception and Sensing
- [ ] Not that relaiable ultrasonic readings
- [ ] Limited 2D vision (no depth perception)
- [ ] ArUco marker dependency for bin detection
- [ ] Limited field of view (no environment scanning)

### Performance
- [ ] Slow object detection (~1 FPS on RPi4) / not obtimized enough
- [ ] Occasional misclassifications
- [ ] Basic state machine architecture
- [ ] Limited Bluetooth range/speed (HC-05)

### Power Management
- [ ] Dual power source complexity
- [ ] No battery monitoring system
- [ ] No low power handling

## 🛣️ Roadmap <a name = "roadmap"></a>
A roadmap for contributors, students, or hobbyists who may want to extend the system and make it more robust.

# Q&A (Roadmap for Improvements)

This section highlights common questions about the project and outlines possible improvements.  
The goal is not to present a final product, but to provide a roadmap for making the system more robust and capable.

---

### Q1. The robotic arm currently uses pre-set positions. Is this enough?

**A1:**  
Pre-set positions are sufficient for a demo or proof of concept, but they limit flexibility.  
Next steps for improvement include:  
- Adding **inverse kinematics (IK)** so the arm can move to any target position dynamically.  
- Using **servo feedback or smart servos** for better precision.  

This would make the robot better at adapting to objects in different positions.

---

### Q2. The project uses only a 2D Pi Camera. Should we add depth sensing?

**A2:**  
A 2D camera works for testing, but without depth perception the robot has trouble estimating distances.  
Improvements:  
- Add a **depth camera** (e.g., Intel RealSense, Orbbec).  
- Or combine the Pi Camera with **distance sensors**.  

This would enable more accurate 3D localization of objects.

---

### Q3. The YOLOv8 model sometimes misclassifies objects. How can detection be improved?

**A3:**  
Ways to improve detection accuracy:  
- Collect more training data from varied environments.  
- Apply data augmentation (brightness, blur, occlusion).  
- Run the model on faster hardware (Coral TPU, Jetson Nano).  
- Add fallback rules (e.g., ignore low-confidence detections).  

These steps would make classification more consistent and reliable.

---

### Q4. Bin detection relies on ArUco markers. Can this be improved?

**A4:**  
ArUco markers are useful for testing but not practical long term.  
Future improvements:  
- Train the system to recognize bins by **shape or color**.  
- Combine ArUco with **visual detection** as a fallback.  

This would reduce reliance on artificial markers and make the system more flexible.

---

### Q5. Navigation is timer-based. What would be a better approach?

**A5:**  
Timer-based navigation is simple but inaccurate. Better approaches include:  
- Adding **wheel encoders** for closed-loop control.  
- Implementing **basic SLAM** using a camera or depth sensor.  
- Adding **bump sensors** as a safety mechanism.  

This would make movement more repeatable and precise.

---

### Q6. Ultrasonic sensors are noisy. Should we replace them?

**A6:**  
Ultrasonic sensors are fine for experiments but can be inconsistent.  
Upgrades could include:  
- Switching to **ToF sensors** (VL53L0X).  
- Using a **depth camera** for obstacle detection.  
- Keeping bump sensors as a backup.  

This would provide more reliable perception.

---

### Q7. The arm struggles with heavier objects. How can we improve strength?

**A7:**  
For light demos, hobby servos are fine, but improvements include:  
- Replace SG90 servos with **metal-gear, higher-torque models**.  
- Add **overcurrent protection** to prevent burnout.  
- Use stronger frame materials (aluminum, reinforced plastic).  

This would make the arm more durable.

---

### Q8. The robot depends on a mobile app for control. Should it be more autonomous?

**A8:**  
Yes. The app is useful for testing, but autonomy should be the next step:  
- Create a **state machine** to automate the cycle: detect → navigate → pick → place.  
- Keep the app for monitoring and manual overrides.  

This would allow the robot to operate independently.

---

### Q9. The project uses multiple separate batteries. Is there a better approach?

**A9:**  
Yes. For simplicity and reliability:  
- Use a **single Li-ion pack** with a Battery Management System (BMS).  
- Add **voltage monitoring** for safe shutdown.  
- Optionally design a **docking/charging system**.  

This reduces wiring complexity and improves stability.

---


### Q10. How can safety be improved?

**A10:**  
Safety matters even in research projects. Improvements include:  
- Add **force limits** for the arm.  
- Include **collision detection** via current sensing.  
- Implement an **emergency stop button**.  

This ensures safe testing and development.

---

### Q11. What if someone wants to build on this project further?

**A11:**  
Future directions could include:  
- Transitioning to **ROS/ROS2** for scalability.  

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
  year={2025},
  publisher={GitHub},
  howpublished={\url{https://github.com/SidCodesRobotics/yolo-robot-picknplace}}
}
``` 
