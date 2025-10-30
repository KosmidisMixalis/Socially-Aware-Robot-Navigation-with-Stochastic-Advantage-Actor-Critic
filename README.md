# Socially-Aware Navigation via Stochastic Advantage Actor-Critic 

**Authors:** Michail Kosmidis, Ioannis Kansizoglou, Antonios Gasteratos  
**Affiliation:** Laboratory of Robotics & Automation (LRA), Department of Production and Management Engineering, Democritus University of Thrace, Xanthi, Greece.  
🔗 [Lab Website](https://robotics.pme.duth.gr)

---

## Overview

This repository contains the full implementation and resources for the paper:

**"Socially-Aware Navigation via Stochastic Advantage Actor-Critic for Continuous Action Control in POMDPs"**

The project integrates reinforcement learning, social force modeling, and ROS-based navigation for a mobile robot operating in human-populated environments.  
Our method builds upon the Ridgeback platform and utilizes a stochastic A2C algorithm to learn continuous action control under partial observability.

---

## Installation

### Prerequisites

- Ubuntu 20.04 / 22.04  
- ROS Noetic / ROS2 Humble  
- Gazebo 11  
- Python 3.8+


## Usage

### 🗺️ Mapping & Localization Example

Save a map:

    roslaunch ridgeback_navigation gmapping_demo.launch
    roslaunch ridgeback_viz view_robot.launch config:=gmapping
    rosrun map_server map_saver -f mymap

Localization:

    roslaunch ridgeback_navigation amcl_demo.launch map_file:=/path/to/mymap.yaml
    roslaunch ridgeback_viz view_robot.launch config:=localization

Gazebo simulation in inference:

    roslaunch ridgeback_gazebo ridgeback_world.launch
    roslaunch ridgeback_navigation amcl_demo.launch map_file:=/home/alien/Desktop/catkin_ws/src/Social_Nav_Src/test/mymap.yaml
    roslaunch ridgeback_viz view_robot.launch config:=localization

---

## 🤖 Included Packages & Dependencies

| Package | Source |
|----------|---------|
| **lightsfm** | https://github.com/robotics-upo/lightsfm.git |
| **gazebo_sfm_plugin** | https://github.com/robotics-upo/gazebo_sfm_plugin.git |
| **ridgeback_related_packages** | Clearpath Robotics : https://github.com/ridgeback|

---




## Citation

If you use this repository, please cite our paper:

    @article{kosmidis2025socially,
      title={Socially-Aware Navigation via Stochastic Advantage Actor-Critic for Continuous Action Control in POMDPs},
      author={Kosmidis, Michail and Kansizoglou, Ioannis and Gasteratos, Antonios},
      year={2025},
      journal={arXiv preprint arXiv:xxxx.xxxxx}
    }

---

## License

This work is licensed under the BSD 3-Clause License.  
See the LICENSE file for details.

---

## Contributing

Contributions are welcome!  

---

## Contact

For questions or collaboration opportunities:

- **Michail Kosmidis** – [mkosmidi@pme.duth.gr]  
- **Laboratory of Robotics & Automation (LRA)** – https://robotics.pme.duth.gr

---

