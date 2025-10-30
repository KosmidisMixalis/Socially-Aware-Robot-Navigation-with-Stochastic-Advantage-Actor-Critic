# Ridgeback ROS Workspace Setup

## Installation Guide (Ubuntu 20.04 + ROS Noetic)

This guide explains how to set up a ROS workspace for simulating the Ridgeback robot on **Ubuntu 20.04 (Focal)** with ROS Noetic.

---

### 1. Prerequisites  
- Ubuntu 20.04 (desktop version recommended)  
- Internet connection for package downloads  
- Sufficient disk space (ROS + simulation packages can take several GB)  

---

### 2. Install ROS Noetic

#### 2.1 Configure Ubuntu repositories  
```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

#### 2.2 Add ROS package repository  
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
  > /etc/apt/sources.list.d/ros-latest.list'
```

#### 2.3 Add ROS GPG key  
```bash
sudo apt install -y curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

#### 2.4 Update and install ROS  
```bash
sudo apt update
sudo apt install -y ros-noetic-desktop-full
```

#### 2.5 Environment setup  
Add the following line to your `~/.bashrc` file so ROS is sourced automatically:
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 2.6 Install build dependencies  
```bash
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator \
  python3-wstool build-essential
```
Initialize `rosdep` (only once):
```bash
sudo rosdep init
rosdep update
```

---

### 3. Verify ROS installation  
In a new terminal run:
```bash
roscore
```
If the ROS master starts without errors, your installation is correct.

---

### 4. Create a Catkin Workspace  
```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 5. Clone Required Packages  
Navigate to your workspace `src` folder:
```bash
cd ~/catkin_ws/src/
```
Clone the necessary repositories, from the links in readme file or see the uploaded workspace:

---

### 6. Install Package Dependencies and Build  
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

---

### 7. Launch Ridgeback Simulation  

#### 7.1 Start Gazebo world  
```bash
roslaunch ridgeback_gazebo ridgeback_world.launch
```

#### 7.2 Start navigation demo  
```bash
roslaunch ridgeback_navigation odom_navigation_demo.launch
```

#### 7.3 View robot in RViz  
```bash
roslaunch ridgeback_viz view_robot.launch config:=navigation
```

---

### 8. Mapping & Localization Example  
```bash
roslaunch ridgeback_navigation gmapping_demo.launch
roslaunch ridgeback_viz view_robot.launch config:=gmapping
rosrun map_server map_saver -f mymap
```
This produces `mymap.yaml` and `mymap.pgm`.  
Then localize:
```bash
roslaunch ridgeback_navigation amcl_demo.launch map_file:=/path/to/mymap.yaml
roslaunch ridgeback_viz view_robot.launch config:=localization
```

---

### 9. Useful Notes  
- Whenever you modify code in `~/catkin_ws/src/`, rerun `catkin_make` or `catkin_make clean && catkin_make`.  
- If you ever wish to clean up unused packages, use:
```bash
sudo apt autoremove
sudo apt clean
```
*Use with caution* — only if you know what you are removing.

---

### 10. Summary  
This guide sets up:  
- Ubuntu 20.04 + ROS Noetic  
- A working catkin workspace for the Ridgeback robot  
- Simulation, mapping and localization workflows  

You’re now ready to begin development and experimentation with Ridgeback!

### 11. Additional Notes

This project uses **TensorFlow**, so it is recommended to enable GPU support following the official guide: [TensorFlow GPU Installation](https://www.tensorflow.org/install/pip).  

The project relies on **ROS packages installed on your system** and **Python libraries managed in a Conda environment**.  

All required Python libraries are listed in the provided `environment.yml` file for easy setup.


