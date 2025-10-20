# Ridgeback ROS Workspace Setup

## Install Ubuntu 20.04 and ROS Noetic

This guide explains how to set up a ROS workspace for simulating the Ridgeback robot using Ubuntu 20.04 and ROS Noetic.


### Prerequisites

Ensure that you have installed Ubuntu 20.04 and ROS Noetic. If not, follow the official ROS installation guide for Ubuntu 20.04.
### Installing ROS Noetic

Once Ubuntu is installed, follow these steps to install **ROS Noetic** visit the [official Ros Noetic download page](http://wiki.ros.org/noetic/Installation/Ubuntu) , the recommended version of ROS compatible with Ubuntu 20.04.

#### Set Up ROS Repository

1. **Add the ROS Package Repository**:  
   Run the following commands to add the ROS package repository to your system:

    ```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    ```

2. **Install `curl`** (if not already installed):

    ```bash
    sudo apt install curl
    ```

3. **Add ROS GPG Keys**:

    ```bash
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    ```

4. **Update the Package List**:

    ```bash
    sudo apt update
    ```

#### Install ROS Noetic

1. **Install the Full Desktop Version of ROS Noetic**:

    ```bash
    sudo apt install ros-noetic-desktop-full
    ```

#### Configure Your Environment

1. **Add ROS Setup Script to `.bashrc`**:  
   This ensures that ROS is automatically sourced in every terminal session.

    ```bash
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

    Or go to the bashrc file and add the following lines at the bottom of the file:

   ```bash
   sudo gedit ~/.bashrc
   source /opt/ros/noetic/setup.bash
   ```

#### Install Dependencies for Building Packages

1. **Install Additional Dependencies**:

    ```bash
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    ```
    
2. **Install rosdep**:

   This will be neccesary for later!
      
   ```bash
   sudo apt install python3-rosdep
   ```
    

2. **Initialize `rosdep`** (necessary for package dependency management):

   Don't execute them yet we will need them later!
   
   ```bash
   sudo rosdep init
   rosdep update
   ```

---

### Verify Installation

1. **Check ROS Installation**:  
   Run the following command to verify that ROS has been installed correctly:

    ```bash
    roscore
    ```

    If ROS is set up correctly, this command will start the ROS master.

---

### Step 1: Create a Catkin Workspace

Follow the instructions in the ROS wiki to [create a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). 

```bash
 source /opt/ros/noetic/setup.bash
mkdir -p ~/Desktop/catkin_ws/src
 cd ~/Desktop/catkin_ws/
catkin_make
```

Source this or put it at the end of ~/.bashrc file
```bash
source /opt/ros/noetic/setup.bash
source ~/Desktop/catkin_ws/devel/setup.bash
```

To apply these changes, either restart the terminal or run:

```bash

source ~/.bashrc
```

### Step 2: Clone Necessary Packages

Navigate to the src folder of your catkin workspace:

```bash

cd ~/Desktop/catkin_ws/src/
```

Then, clone the required packages inside src:

navigation
realsense_ros_gazebo 
ridgeback
ridegeback desktop
ridgeback simulator


### Step 3: Install Packages

Run the following commands to install the packages:

```bash
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

### Step 4: Start Gazebo Ridgeback Robot Simulation

Launch the Gazebo model:

```bash

roslaunch ridgeback_gazebo ridgeback_world.launch
```

Launch the navigation:

```bash

roslaunch ridgeback_navigation odom_navigation_demo.launch
```

Visualize the RViz configuration:

```bash

roslaunch ridgeback_viz view_robot.launch config:=navigation
```
in order to control the robot through simulation + rviz

you must either point it in a direction with nav2d 
or with rqt command in terminal



Navigating Ridgeback

To get all Navigation related files for Ridgeback, run:

sudo apt-get install ros-noetic-ridgeback-navigation

roslaunch ridgeback_navigation gmapping_demo.launch
roslaunch ridgeback_viz view_robot.launch config:=gmapping
rosrun map_server map_saver -f mymap
This will create a mymap.yaml and mymap.pgm file in your current directory.
roslaunch ridgeback_navigation amcl_demo.launch map_file:=/path/to/my/map.yaml
roslaunch ridgeback_viz view_robot.launch config:=localization

Useful Note everytime you change something in src and to re catkin_make first run catkin_make clean
sudo apt remove $(comm -23 <(apt-mark showmanual | sort) <(gzip -dc /var/log/installer/initial-status.gz | sed 's/^[^ ]* //g' | sort))

In Summary:

This command removes all manually installed packages that weren’t part of the initial installation. This helps in cleaning up packages that may have been manually added later but are no longer needed, leaving your system with just the essential packages that were there at the start.

sudo apt autoremove
sudo apt clean


