# Collaborative ABB Robot as an opponent in the Air Hockey Game 🤖🏒

**Master's Thesis Project**  
Faculty of Applied Informatics, Tomas Bata University in Zlín  
**Author**: Vojtěch Birgus  
**Thesis supervisor**: Ľuboš Spaček  
**Year**: 2025


---

## 🧠 Overview

This repository contains the implementation of a collaborative robotic system designed to play air hockey against a human opponent. The project combines computer vision, object tracking, trajectory prediction, and real-time robot control using ABB's Externally Guided Motion (EGM) interface.

The system uses a vision pipeline to detect the puck and predict its motion. Based on the predicted trajectory, the robot reacts either defensively or offensively to intercept or hit the puck back.

![porovnani-realsimulace_new2](https://github.com/user-attachments/assets/8c614121-4ddf-4090-8dc8-869a9086f6de)




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



https://github.com/user-attachments/assets/263d3c55-cbe8-4137-a946-d892f6663bf2

https://github.com/user-attachments/assets/2b463e1e-2f4c-4144-8117-56780a42aeff

https://github.com/user-attachments/assets/e11c11ee-a758-47f0-a9bd-0699d7d55edf




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



---

## 📬 Contact

For academic inquiries or collaboration:  
**Vojtěch Birgus**  
[GitHub Profile](https://github.com/vbirgus)
