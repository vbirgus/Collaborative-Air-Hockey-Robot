# Collaborative Air Hockey Robot 🤖🏒

**Master's Thesis Project**  
Faculty of Applied Informatics, Tomas Bata University in Zlín  
**Author**: Bc. Vojtěch Birgus  
**Year**: 2025

---

## 🧠 Overview

This repository contains the implementation of a collaborative robotic system designed to play air hockey against a human opponent. The project combines computer vision, object tracking, trajectory prediction, and real-time robot control using ABB's Externally Guided Motion (EGM) interface.

The system uses a vision pipeline to detect the puck and predict its motion. Based on the predicted trajectory, the robot reacts either defensively or offensively to intercept or hit the puck back.

---

## 🔧 Key Features

- 🎯 Real-time puck detection and tracking using OpenCV
- 📈 Trajectory prediction algorithm with bounce handling
- 🧭 Perspective correction based on ArUco marker detection
- 🤖 Support for both simulation and real robot via ABB RobotStudio + EGM
- 🖼️ Visual debugging and mask display during gameplay

---

## 💻 Technologies Used

- Python 3
- OpenCV (aruco, image processing)
- NumPy
- Pillow (for screen capture in simulation)
- pypylon (Basler camera SDK)
- ABB EGM SDK (socket-based communication)

---

## 📁 Repository Structure

```
├── ABBRobotEGM_lib/ # Python code using ABBRobotEGM_lib library
├── EGM_PB2_lib/ # Old python code using EGM_PB2_lib
├── RobotStudio/ # Rspag file for simulation purposes
├── findcolor.py # Script for hsv color find
├── README.md # This file └── ...
```

---

## ⚙️ How to Run

### Requirements

Install dependencies:
```bash
pip install opencv-python numpy pypylon Pillow
```

### Hardware Setup
- Connect a Basler camera
- Setup ABB robot with RobotWare and enable EGM communication
- Calibrate the table using ArUco markers (IDs 0,1,2,3)

### Execution

```bash
python v5_egm_ABBRobotEGM_lib.py
```

- If ArUco markers are not detected, a manual ROI selection will be prompted.
- For simulation, set `camera_real = False` in the script.

---

## 🎥 Demonstration

Images, animations, or demo videos showcasing the robot's actions and vision system can be added here.
https://github.com/user-attachments/assets/5be29a45-7db5-46cd-9c55-678b50e69f5a

---

## 📘 Academic Context

This project is part of a master's thesis at the Faculty of Applied Informatics, Tomas Bata University in Zlín. The theoretical part of the thesis covers:

- 🏓 Air hockey rules and player strategies
- 🧠 Computer vs. machine vision
- 🛠️ Vision system design using OpenCV
- 🤝 Collaborative vs. industrial robots
- 🧩 ABB RobotStudio environment and EGM communication

---

## 📜 License

This academic project is subject to the Czech copyright law §60 of Act No. 121/2000 Coll. as a university work. The usage of the code or its components for commercial purposes is restricted without prior written consent from Tomas Bata University in Zlín.

---

## 📬 Contact

For academic inquiries or collaboration:  
**Vojtěch Birgus**  
[GitHub Profile](https://github.com/vbirgus)
