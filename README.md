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

---

## Folder Structure

catkin_ws/
└── src/
    └── Social_Nav_Src/
        ├── train/       # Contains training scripts and datasets
        └── test/        # Contains testing scripts and evaluation data
        

> ⚠️ Some files in `catkin_ws/src/ridgeback_simulator/ridgeback_gazebo/Media` and Python scripts may require path replacements.

---

## Installation

### Prerequisites

- Ubuntu 20.04  
- ROS Noetic  
- Gazebo 11  
- Python 3.8+  

---

## Included Packages & Dependencies

| Package | Source |
|---------|--------|
| **lightsfm** | [GitHub](https://github.com/robotics-upo/lightsfm.git) |
| **gazebo_sfm_plugin** | [GitHub](https://github.com/robotics-upo/gazebo_sfm_plugin.git) |
| **ridgeback_related_packages** | Clearpath Robotics: [GitHub](https://github.com/ridgeback) |

---

## Usage

### Training

1. In one terminal, launch the simulation:
    ```bash
    roslaunch ridgeback_gazebo ridgeback_world.launch
    ```
2. In a second terminal, navigate to the training folder:
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
2. In another terminal, start localization:
    ```bash
    roslaunch ridgeback_navigation amcl_demo.launch map_file:=/your_path/catkin_ws/src/Social_Nav_Src/test/mymap.yaml
    ```
3. In a third terminal, visualize the robot:
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

6. **Scenario-specific configuration** (edit line 108 in `Test_Agent.py`):
    - Scenario 1:
      ```python
      self.humans_points.append([get_model_pose("person"), get_model_pose("person2")])
      ```
    - Scenario 2:
      ```python
      self.humans_points.append([get_model_pose("person3"), get_model_pose("person4"), get_model_pose("person5")])
      ```
    - Scenario 3:
      ```python
      self.humans_points.append([get_model_pose("actor1")])
      ```

> ⚠️ For cleaning distance, run repetitions `1` time `10` and use the average, due to a known bug.  
> Adjust paths in scripts if necessary based on your local setup.

---

## Contributing

Contributions are welcome! Please submit issues or pull requests for improvements, bug fixes, or new features.

---

## Contact

For questions or collaboration opportunities:

- **Michail Kosmidis** – [mkosmidi@pme.duth.gr](mailto:mkosmidi@pme.duth.gr)

---

> ⚡ We are currently preparing a **clean and fully up-to-date setup file** that will allow users to install all required dependencies and packages from scratch.

