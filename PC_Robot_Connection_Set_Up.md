# Setting Up Remote Computer → Ridgeback Robot Connection

This guide provides step-by-step instructions to connect a remote desktop computer to the Clearpath Ridgeback robot via Ethernet (wired) or WiFi (wireless). It also covers installing Ubuntu 20.04 and ROS Noetic on your development machine, and configuring ROS for remote use with Ridgeback.  
Based on Clearpath’s official documentation.  
🔗 [Clearpath Ridgeback Networking Guide](https://docs.clearpathrobotics.com/docs/ros1noetic/robots/indoor_robots/ridgeback/tutorials_ridgeback/)

---

## Table of Contents

1. [Set up Remote Computer OS & ROS](#set-up-remote-computer-os--ros)  
   - [Install Ubuntu 20.04 LTS](#install-ubuntu-2004-lts)  
   - [Install ROS Noetic](#install-ros-noetic)  
     - [Set Up ROS Repository](#set-up-ros-repository)  
     - [Install ROS Noetic](#install-ros-noetic-1)  
     - [Configure Environment](#configure-environment)  
     - [Install Build Dependencies](#install-build-dependencies)  
   - [Verify Installation](#verify-installation)  

2. [Connect to Ridgeback](#connect-to-ridgeback)  
   - [Ethernet Cable Connection](#ethernet-cable-connection)  
   - [Wireless (WiFi) Connection](#wireless-wifi-connection)  
     - [Step 1: Create WiFi netplan Configuration](#step-1-create-wifi-netplan-configuration)  
     - [Step 2: Apply Netplan Configuration](#step-2-apply-netplan-configuration)  
     - [Step 3: Set ROS_MASTER_URI and ROS_IP](#step-3-set-ros_master_uri-and-ros_ip)  
     - [Step 4: Test Remote ROS Connectivity](#step-4-test-remote-ros-connectivity)  

3. [Useful Notes](#useful-notes)  

---

## Set up Remote Computer OS & ROS

### Install Ubuntu 20.04 LTS
Install Ubuntu 20.04 (desktop version recommended) on your remote computer. Ensure internet connectivity.

### Install ROS Noetic

#### Set Up ROS Repository
```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

#### Add ROS GPG Key & Update
```bash
sudo apt install -y curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
```

#### Install ROS Noetic
```bash
sudo apt install -y ros-noetic-desktop-full
```

#### Configure Environment
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Install Build Dependencies
```bash
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

---

### Verify Installation
In a fresh terminal, run:
```bash
roscore
```
If ROS master starts without error, installation is correct.

---

## Connect to Ridgeback

### Ethernet Cable Connection
1. Connect your computer’s Ethernet port to the Ridgeback’s Ethernet port.  
2. Assign a static IP on your computer in the `192.168.131.x` subnet, e.g.:  
   - IP: `192.168.131.100`  
   - Netmask: `255.255.255.0`  
   - Gateway: none or `0.0.0.0`  
3. SSH into Ridgeback:
```bash
ssh administrator@192.168.131.1
```
Default password: `clearpath` (change this after login).

---

### Wireless (WiFi) Connection

#### Step 1: Create WiFi netplan Configuration
SSH into the Ridgeback via Ethernet, then:
```bash
sudo nano /etc/netplan/60-wireless.yaml
```
Paste and edit:
```yaml
network:
  wifis:
    <WIRELESS_INTERFACE>:           # e.g., wlan0 or wlp2s0
      optional: true
      access-points:
        "SSID_GOES_HERE":
          password: "PASSWORD_HERE"
      dhcp4: true
      dhcp4-overrides:
        send-hostname: true
```

#### Step 2: Apply Netplan Configuration
```bash
sudo netplan apply
```
Check the assigned IP:
```bash
ip a
```

#### Step 3: Set ROS_MASTER_URI and ROS_IP
On the remote computer, find its IP:
```bash
ip a
```
Then create a script `remote-robot.sh`:
```bash
export ROS_MASTER_URI=http://<Ridgeback_HOSTNAME_OR_IP>:11311
export ROS_IP=<Remote_Computer_Wireless_IP>
```
If hostname is not resolved, edit `/etc/hosts`:
```
<Ridgeback_IP> <Ridgeback_HOSTNAME>
```
Then source the script:
```bash
source ~/remote-robot.sh
```

#### Step 4: Test Remote ROS Connectivity
```bash
rostopic list
```
You should see topics published by Ridgeback’s ROS master.  
Optionally, launch visualization:
```bash
roslaunch ridgeback_viz view_robot.launch
```

---

## Useful Notes
- Robot and remote computer must be on the same network (wired or wireless).  
- Change the default password (`clearpath`) for security.  
- Restart networking or logout/login if configuration changes.  
- Avoid altering core network bridge unless advanced.  
- If ROS topics do not appear, check `ROS_MASTER_URI`, `ROS_IP`, firewall, and network reachability.

---

Following these steps, your remote computer will be properly connected to the Ridgeback robot’s ROS system for monitoring, visualization, and control.
