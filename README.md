# Broverette Nav2 Project

![AckbotAutonomousNavigation_TestingROS2Nav2PathPlannerwithAMCL-ezgif com-optimize](https://github.com/user-attachments/assets/a0f7e0b2-b9ab-4402-a0fb-24f9cc5b6f2a)

https://github.com/user-attachments/assets/03323889-74ca-40c9-b6ba-a75158bd82f6

## Overview
The Broverette project provides the necessary code and configurations to control and navigate a robot using a Raspberry Pi 4 (Broverette) and a laptop running Ubuntu 22 with ROS 2 Humble. Broverette itself handles the basic sensor and movement operations, such as publishing topics like `cmd_vel`, `odom`, and `scan`. However, all higher-level functions, such as mapping (SLAM), localization (AMCL), and navigation (Nav2), are handled on the laptop, which subscribes to these topics.

### Key Points:

- **Broverette (Raspberry Pi 4)**: Runs basic ROS 2 nodes, publishing necessary data to topics for movement and sensing (cmd_vel, odom, scan).
- **Laptop (Ubuntu 22, ROS 2 Humble)**: Handles more computationally intensive tasks, such as controlling Broverette with a PS4 controller, running SLAM for mapping, and using AMCL and Nav2 for path planning and navigation.
- **Multi-Machine Setup**: Both the Raspberry Pi 4 and the laptop are set to the same ROS_DOMAIN_ID to ensure proper communication across the network.

This repository focuses on setting up and running the SLAM and navigation tasks from the laptop, with control over Broverette via a PS4 controller. All the tasks involving teleoperation, SLAM, saving the map, and performing autonomous navigation are executed on the laptop while Broverette serves as the robot's hardware platform.

### Table of Contents:
1. [Dependencies](#dependencies)
2. [Setup](#setup)
3. [Running Broverette](#running-broverette)
4. [Performing SLAM and Saving Maps](#performing-slam-and-saving-maps)
5. [Running AMCL and Navigation](#running-amcl-and-navigation)
   
## Dependencies

- **ROS 2 Humble**: The laptop uses Ubuntu 22 and ROS 2 Humble to run SLAM, AMCL, and navigation.
- **Broverette's Pi 4**: Broverette is powered by a Raspberry Pi 4, which publishes the topics `cmd_vel`, `odom`, and `scan` to which the laptop subscribes.
- **Nav2**: For path planning and navigation, which runs on the laptop.
- **SLAM Toolbox**: For scanning the room and generating maps (runs on the laptop).
- **PS4 Controller Node**: For teleoperating Broverette using the PS4 controller (also controlled from the laptop).

## Prerequisites

Ensure you have ROS 2 Humble installed and sourced in your environment. If SLAM Toolbox or Nav2 are not already installed, follow the steps below:
### Install SLAM Toolbox

SLAM Toolbox provides mapping and localization functionalities in ROS 2. To install it, use the following command:
```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox
```

### Install Nav2 

Nav2 allows you to perform path planning, obstacle avoidance, and autonomous navigation. Install it by running:
```bash
sudo apt update
sudo apt install ros-humble-nav2-bringup
```


## Setup

### Clone the Repository
To set up the Broverette navigation stack, first create a new ROS 2 workspace on the **laptop** if you haven't already:

```bash
mkdir ~/broverette_nav2_ws/
cd ~/broverette_nav2_ws/
git clone https://github.com/TheTacoBytes/Broverette_Nav2.git .
```

### Build the Workspace

After cloning the repository, navigate to the workspace root and build it using `colcon`:
```bash
cd ~/broverette_nav2_ws
colcon build
```

### Source the Workspace

After building, make sure to source the workspace on the laptop:

```bash
source install/setup.bash
```

## Running SLAM and Saving Maps
The following steps will require the Broverette to be powered on and connected to same Wi-Fi as the Laptop. 

### 1. Multi-Machine Required
First, In order for the laptop to see the topics you need to make sure you have set both Broverette and Laptop on the same `ROS_DOMAIN_ID`. For example, you can set it to `1` by running:

```bash
export ROS_DOMAIN_ID=1
```
Now check to see if the Laptop can see the topics from the Broverette's Pi by running:
```bash
ros2 topic list
```
There you should see `cmd_vel`,`scan`,`odom`, ect.

### 2. Launch the PS4 Controller Node to Control Broverette Manually

Next, you need to launch the PS4 controller node to teleoperate Broverette for scanning the environment.
```bash
ros2 launch ps4_controller controller_launch.py
```
### 3. Launch the SLAM Node to Perform Scanning
Once you have control of Broverette, run the SLAM node on your laptop to scan the environment:
```bash
ros2 launch broverette_slam slam_launch.py
```
This will initiate SLAM, and Broverette will start mapping the environment. Use the PS4 controller to move Broverette around to create a complete map.

Once the scanning is complete, use the **SLAM panel** in your interface to save the map, so you dont have to edit the code reuse the name `map`. The map is saved as `map.pgm` and `map.yaml` on the **laptop**.

### 4. Move the Map for Navigation  
Move the saved map files to the appropriate folder for Nav2 to use:
```bash
cd ~/broverette_nav2_ws
mv map.pgm map.yaml ~/broverette_nav2_ws/src/broverette_nav2_bringup/maps/
```
You can now cancel the controller process and the slam process from running.

Once finished you will need to rebuild the workspace and source it one more time.
After cloning the repository, navigate to the workspace root and build it using `colcon`:
```bash
cd ~/broverette_nav2_ws
colcon build
source install/setup.bash
```

## Running AMCL and Navigation
### Localization with AMCL 
To localize Broverette in the environment using the saved map, run the following command on the **laptop**:
```bash
ros2 launch broverette_nav2_bringup localization_launch.py use_sim_time:=false
```
Now, based on Broverette's actual position in the real world, set a **2D Pose Estimate** in RVIZ on the map. If the **map** doesn't appear in RVIZ, check the map dropdown in the Displays panel. Locate the topic set to `/map`, change it to any other value, and then switch it back to `/map`. Next, go to the Topic dropdown and set the **Durability Policy** to `Transient Local`.

### Path Planning and Navigation
Once Broverette is localized, you can run Nav2 for path planning and navigation on the **laptop**:
```bash
ros2 launch broverette_nav2_bringup navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true
```

### Set Goal Pose
In RVIZ click 2D Goal Pose and set the position and direction and watch as the Broverette moves.

## Viewing the Webcam feed
**Run the Image Subscriber on the Laptop**: On your laptop, run the image subscriber node to view the webcam feed:
```bash
ros2 run webcam_feed img_subscriber
```
You should see the video stream in an OpenCV window.

