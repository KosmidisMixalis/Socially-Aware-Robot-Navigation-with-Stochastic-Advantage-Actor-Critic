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

#### 2.1 Configure Ubuntu Repositories  
```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

#### 2.2 Add ROS Package Repository  
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

#### 2.3 Add ROS GPG Key  
```bash
sudo apt install -y curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

#### 2.4 Update and Install ROS  
```bash
sudo apt update
sudo apt install -y ros-noetic-desktop-full
```

#### 2.5 Environment Setup  
Add the following lines to your `~/.bashrc` to source ROS automatically:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 2.6 Install Build Dependencies  
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

### 3. Verify ROS Installation  

Open a new terminal and run:
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
Or everytime you build the workspace execute
# Source your workspace
```source ~/catkin_ws/devel/setup.bash```


---

### 5. Clone Required Packages  

Navigate to your workspace `src` folder:
```bash
cd ~/catkin_ws/src/
```
Clone the necessary repositories as described in the project README or use the uploaded workspace.

---

### 6. Install Package Dependencies and Build  
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

---

### 7. Launch Ridgeback Simulation  

#### 7.1 Start Gazebo World  
```bash
roslaunch ridgeback_gazebo ridgeback_world.launch
```

#### 7.2 Start Navigation Demo  
```bash
roslaunch ridgeback_navigation odom_navigation_demo.launch
```

#### 7.3 View Robot in RViz  
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
This generates `mymap.yaml` and `mymap.pgm`.  

Then localize the robot:
```bash
roslaunch ridgeback_navigation amcl_demo.launch map_file:=/path/to/mymap.yaml
roslaunch ridgeback_viz view_robot.launch config:=localization
```

---

### 9. Useful Notes  

- Whenever you modify code in `~/catkin_ws/src/`, rerun:
```bash
catkin_make
# or for a full rebuild
catkin_make clean && catkin_make
```
- To clean up unused packages:
```bash
sudo apt autoremove
sudo apt clean
```
*Use with caution — only remove packages you know are unnecessary.*

---

### 10. Summary  

This guide sets up:  
- Ubuntu 20.04 + ROS Noetic  
- A working catkin workspace for the Ridgeback robot  
- Simulation, mapping, and localization workflows  

You are now ready to begin development and experimentation with Ridgeback!

---

### 11. Additional Notes

- This project uses **TensorFlow**, so it is recommended to enable GPU support following the official guide: [TensorFlow GPU Installation](https://www.tensorflow.org/install/pip).  
- The project relies on **ROS packages installed on your system** and **Python libraries managed in a Conda environment**.  
- All required Python libraries are listed in the provided `environment.yml` file for easy setup.

---

### 12. Source ROS in `.bashrc`  

To ensure your ROS environment is sourced automatically, open your `.bashrc`:
```bash
gedit ~/.bashrc
```
Add the following lines at the end of the file:

```bash
# >>> ROS Noetic initialize >>>
source /opt/ros/noetic/setup.bash
source ~/Desktop/catkin_ws/devel/setup.bash
# <<< ROS Noetic initialize <<<
```
Save and close. Then apply:
```bash
source ~/.bashrc
```

