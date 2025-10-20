# Setting up the connection between a Remote Computer to Clearpath Ridgeback Robot

This comprehensive guide provides step-by-step instructions to connect a remote desktop computer to the Clearpath Ridgeback robot via Ethernet (wired) and WiFi (wireless). Additionally, it covers the installation of necessary software and ROS Noetic on your development machine, ensuring smooth communication and control of the Ridgeback robot. For this integration guide we will follow the guide provided by [clearpath's official page](https://docs.clearpathrobotics.com/docs/ros1noetic/robots/indoor_robots/ridgeback/tutorials_ridgeback/)

## Table of Contents
1. [Installing Remote Computer OS](#installing-remote-computer-os)
   - [Installing Ubuntu 20.04 LTS](#installing-ubuntu-2004-lts)
   - [Installing ROS Noetic](#installing-ros-noetic)
     - [Set Up ROS Repository](#set-up-ros-repository)
     - [Install ROS Noetic](#install-ros-noetic)
     - [Configure Your Environment](#configure-your-environment)
     - [Install Dependencies for Building Packages](#install-dependencies-for-building-packages)
   - [Verify Installation](#verify-installation)
2. [Connecting to Ridgeback](#connecting-to-ridgeback)
   - [Connection via Ethernet Cable](#connection-via-ethernet-cable)
   - [Connection via Wireless (WiFi)](#connection-via-wireless-wifi)
     - [Step 1: Create a WiFi Configuration File](#step-1-create-a-wifi-configuration-file)
     - [Step 2: Apply the Netplan Configuration](#step-2-apply-the-netplan-configuration)
     - [Step 3: Setting ROS_MASTER_URI and ROS_IP](#step-3-setting-ros_master_uri-and-ros_ip)
     - [Step 4: Testing Remote ROS Connectivity](#step-4-testing-remote-ros-connectivity)
3. [Useful Notes](#useful-notes)

---


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

## Connecting to Ridgeback

Once your remote computer is set up with Ubuntu and ROS Noetic, you can connect it to the Ridgeback robot using either an Ethernet cable or WiFi.

### Connection via Ethernet Cable

1. **Physical Connection**:  
   Connect your desktop computer to the Ridgeback robot's onboard computer using an Ethernet cable through the Ethernet port on the robot’s side panel.

2. **Set Static IP on Your Computer**:  
   Assign a static IP address to your computer's Ethernet port:
   
   - go to network settings.
   - Under **Ethernet**, select your wired connection and click **Edit**.
   - Go to the **IPv4 Settings** tab and set the **Method** to **Manual**.
   - Add a new address:  
     - IP: `192.168.131.101`  
     - Netmask: `255.255.255.0`
     - Port: `0.0.0.0`
   - Save the settings.
   - Now connecting your remote pc and robot with ethernet it will manually establish a connection between them.

3. **SSH into Ridgeback**:  
   Open a terminal on your remote computer and run the following command:

    ```bash
    ssh administrator@192.168.131.1 
    ```

4. **Login**:  
   Enter the default password: `clearpath`.

---

### Connection via Wireless (WiFi)

To connect wirelessly to the Ridgeback, follow these steps:

#### Step 1: Create a WiFi Configuration File

1. **SSH into Ridgeback via Ethernet**:  
   First, establish a connection using the Ethernet method described above.

2. **Navigate to the Netplan Directory**:  
   Navigate to the Netplan directory:

    ```bash
    cd ..
    cd /etc/netplan/
    ```

3. **Create a New WiFi Configuration File**:

    ```bash
    sudo nano /etc/netplan/60-wireless.yaml
    ```

4. **Add the WiFi Configuration**:  
   Paste the following YAML configuration into the file and adjust it according to your WiFi network:

    ```yaml
    network:
      wifis:
        wireless_interface: <replace_with_your_wireless_interface>  # e.g., wlp2s0
        optional: true
        access-points:
          "SSID_GOES_HERE":
            password: "password_here"
        dhcp4: true
    ```
    
    Replace WIRELESS_INTERFACE with the name of the wireless network device, e.g. wlan0 or wlp3s0
    Fill in the SSID and PASSWORD fields as appropriate.  The password may be included as plain-text
    or as a password hash.  To generate the hashed password, run
    echo -n 'WIFI_PASSWORD' | iconv -t UTF-16LE | openssl md4 -binary | xxd -p
    If you have multiple wireless cards you may include a block for each device.
    For more options, see https://netplan.io/reference/

   Replace the placeholders with the appropriate network details.

#### Step 2: Apply the Netplan Configuration

1. **Apply the Configuration**:

    ```bash
    sudo netplan apply
    ```

2. **Verify the WiFi Connection**:  
   Use the following command to check if Ridgeback is connected to WiFi:

    ```bash
    ssh administrator@<robot's wireless ip/ the inet ip of the robot when you hit {ip a}>
    ```
   Be aware that the the wireless ip of robot may change if other robot devices are connected. 
   The IP address assigned to a robot on a local network (usually by a DHCP server, such as a router) can change under certain circumstances.
    

#### Step 3: Setting ROS_MASTER_URI and ROS_IP

1. **Use the `ip` Command**:  
   Run the following command on your Remote Computer to find its wireless IP address:

    ```bash
    ip a
    ```

    Look for the IP address under the network interface starting with `w` (for wireless) or `en` (for Ethernet). It might look like:

    ```bash
    inet 10.25.0.102/24
    ```

2. **Create a ROS Connection Script**:  
   For easyness, Create a file named `remote-robot.sh` in your home directory in your remote pc with the following content:

    ```bash
    export ROS_MASTER_URI=http://cpr-robot-0001:11311  # Replace with your robot's hostname
    export ROS_IP=10.25.0.102  # Replace with your Remote Computer's Wireless IP address
    ```

3. **Edit `/etc/hosts` (Optional)**:  
   If your network does not resolve the robot's hostname automatically, add an entry to the `/etc/hosts` file on your Remote Computer:

   Open file hosts:

   ```bash
   sudo nano /etc/hosts
   ```
   
   It should be like this:
   
    ```bash
    X.X.X.X       localhost
    X.X.X.X       usr
    robot's wireless ip address        cpr-r100-0562

    ```

5. **Source the Script**:  
   When you're ready to connect to the robot, run the following command to load the environment variables:

    ```bash
    source ~/remote-robot.sh
    ```

### Step 4: Testing Remote ROS Connectivity

1. **Verify ROS Communication**:  
   On the remote computer, run a simple ROS command to verify that it can communicate with the Ridgeback robot's ROS master:

    ```bash
    rostopic list
    ```

    This should return a list of active topics on the Ridgeback robot if everything is set up correctly.

2. **Launch RViz for Visualization**:  
   Try launching RViz, the standard ROS visualization tool, to confirm the connection:

    ```bash
    roslaunch ridgeback_viz view_robot.launch
    ```

### Conclusion

By following these steps, you should be able to remotely connect your computer to the Ridgeback robot's ROS master and execute ROS commands or visualization tools like RViz.

---

## Useful Notes

- Ensure both the Ridgeback robot and your remote computer are on the same network when connecting via WiFi.
- You may need to modify firewall settings if connectivity issues occur.

---
