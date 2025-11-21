# Socially-Aware Navigation via Stochastic Advantage Actor-Critic 

**Authors:** Michail Kosmidis, Ioannis Kansizoglou, Antonios Gasteratos  
**Affiliation:** Laboratory of Robotics & Automation (LRA), Department of Production and Management Engineering, Democritus University of Thrace, Xanthi, Greece  
🔗 [Lab Website](https://robotics.pme.duth.gr)

---

## Overview

This repository contains the full implementation and resources for the paper:

**"Socially-Aware Navigation via Stochastic Advantage Actor-Critic for Continuous Action Control in POMDPs"**

The project integrates reinforcement learning, social force modeling, and ROS-based navigation for a mobile robot operating in human-populated environments.  
Our method builds upon the Ridgeback platform and utilizes a stochastic A2C algorithm to learn continuous action control under partial observability.

> ⚠️ The work in this repository is created for academic research. All rights of external packages are reserved to their original owners as per their licenses. Everything in `Social_Nav_Src` is developed by the authors and is open-source for academic use.

Some files like `catkin_ws/src/ridgeback_simulator/ridgeback_gazebo/Media` and Python scripts may require **path replacements** based on your local setup.

---

## Installation

### Prerequisites

- Ubuntu 20.04  
- ROS Noetic  
- Gazebo 11  
- Python 3.8+  

---

## Included Packages & Dependencies

This project includes or modifies the following open-source packages. All original copyright
notices and licenses are retained in their respective folders.

| Package | Source | License / Notes |
|---------|--------|----------------|
| **lightsfm** | [GitHub](https://github.com/robotics-upo/lightsfm.git) | BSD 3-Clause |
| **gazebo_sfm_plugin** | [GitHub](https://github.com/robotics-upo/gazebo_sfm_plugin.git) | BSD 3-Clause |
| **Ridgeback related packages** | Clearpath Robotics: [GitHub](https://github.com/ridgeback) | BSD 3-Clause |
| **Navigation** | [GitHub](https://github.com/ros-planning/navigation) | Not included in this repo; clone separately from GitHub |
| **RealSense Camera packages** | [GitHub](https://github.com/nilseuropa/realsense_ros_gazebo) | No license; not included in this repo; clone separately from GitHub |

> ⚡ Only packages with permissive licenses are included in this repository. Packages without a license are referenced but not redistributed.

---

## Usage

### Training

1. Launch the simulation:
    ```bash
    roslaunch ridgeback_gazebo ridgeback_world.launch
    ```
2. Navigate to the training folder:
    ```bash
    cd catkin_ws/src/Social_Nav_Src/train
    ```
3. Run the training script:
    ```bash
    python3 main.py
    ```

---

## Pre-trained Models

- `Pretrain_A2C` contains a trained A2C model.

---

## Contact

For questions or collaboration opportunities:

- **Michail Kosmidis** – [mkosmidi@pme.duth.gr](mailto:mkosmidi@pme.duth.gr)

---

> ⚡ We are preparing a clean and fully up-to-date setup, for Ros2.
