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

### Testing

1. Launch the simulation:
    ```bash
    roslaunch ridgeback_gazebo ridgeback_world.launch
    ```
2. Start localization:
    ```bash
    roslaunch ridgeback_navigation amcl_demo.launch map_file:=/your_path/catkin_ws/src/Social_Nav_Src/test/mymap.yaml
    ```
3. Visualize the robot:
    ```bash
    roslaunch ridgeback_viz view_robot.launch config:=localization
    ```
4. Navigate to the test folder:
    ```bash
    cd catkin_ws/src/Social_Nav_Src/test
    ```
5. Run the evaluation script:
    ```bash
    python3 Test_Agent.py --tests <x> --scenario <y> --algo <name>
    ```
    - `<x>`: Number of tests (e.g., 10)  
    - `<y>`: Scenario number (`1`, `2`, or `3`)  
    - `<name>`: Algorithm (`dwa` or `a2c`)  

6. Scenario-specific configuration (edit line 108 in `Test_Agent.py`):
    - **Scenario 1**
      ```python
      self.humans_points.append([get_model_pose("person"), get_model_pose("person2")])
      ```
    - **Scenario 2**
      ```python
      self.humans_points.append([get_model_pose("person3"), get_model_pose("person4"), get_model_pose("person5")])
      ```
    - **Scenario 3**
      ```python
      self.humans_points.append([get_model_pose("actor1")])
      ```

> ⚠️ For cleaning distance, run repetitions `1` time `10` and use the average, due to a known bug. Adjust paths if necessary.

---

## Pre-trained Models

- `ActorCriticTest/Test` contains a trained A2C model.

---

## Contributing

Contributions are welcome! Submit issues or pull requests for improvements, bug fixes, or new features.

> ⚠️ Ensure all packages are under `catkin_ws/src` and check file paths in scripts like training scripts and models in `ridgeback_simulator/ridgeback_gazebo/Media/models/person_standing`.

---

## Contact

For questions or collaboration opportunities:

- **Michail Kosmidis** – [mkosmidi@pme.duth.gr](mailto:mkosmidi@pme.duth.gr)

---

> ⚡ We are preparing a clean and fully up-to-date setup file that will allow users to install all required dependencies and packages from scratch.
