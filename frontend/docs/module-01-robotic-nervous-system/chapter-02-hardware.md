---
sidebar_position: 3
---

# URDF for Humanoids and Hardware Setup

URDF (Unified Robot Description Format) is essential for defining humanoid robot models. This chapter covers both URDF for humanoids and hardware setup with Jetson Orin and RealSense cameras.

## URDF for Humanoid Robots

Unified Robot Description Format (URDF) is an XML-based format used to describe robotic systems in ROS. For humanoid robots, URDF definitions must account for complex kinematic chains that mimic human structure.

### Humanoid-Specific URDF Considerations

Humanoid robots require special attention in their URDF definitions:

- **Kinematic Chains**: Multiple limbs with specific joint constraints (spherical joints for shoulders, revolute joints for elbows)
- **Balance**: Proper mass distribution and inertial properties for bipedal stability
- **Degrees of Freedom**: Humanoids typically have 20+ joints requiring precise definition

### Basic Humanoid URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Torso/Body -->
  <link name="torso">
    <visual>
      <geometry>
        <capsule radius="0.1" length="0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <capsule radius="0.1" length="0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.3"/>
  </joint>

  <!-- Arms, legs, and other limbs would continue with similar definitions -->

  <!-- Left Arm Example -->
  <link name="left_shoulder">
    <visual>
      <geometry>
        <capsule radius="0.05" length="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <capsule radius="0.05" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="spherical">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0.0 0.15 0.2"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

## Hardware Setup for Humanoid Control

The Jetson Orin and Intel RealSense cameras form a powerful combination for humanoid robotic control applications, providing high-performance edge computing and accurate depth perception capabilities.

## Jetson Orin Setup

The NVIDIA Jetson Orin is a powerful edge AI computing platform that serves as the brain for many robotic systems. Here's how to set it up:

### Initial Setup
1. Connect the Jetson Orin to a monitor, keyboard, and mouse
2. Power on the device and follow the initial Ubuntu setup
3. Connect to Wi-Fi or Ethernet for internet access
4. Update the system: `sudo apt update && sudo apt upgrade`

### Installing ROS 2
```bash
# Add ROS 2 GPG key and repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
```

### Jetson-specific optimizations
```bash
# Install Jetson.GPIO for hardware interfacing
sudo apt install python3-jetson-gpio

# Install Jetson Stats for monitoring
sudo -H pip3 install -U jetson-stats
```

## Intel RealSense Setup

The Intel RealSense camera provides RGB-D (color and depth) data essential for robotics perception tasks.

### Installation
```bash
# Add RealSense repository
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# Install RealSense packages
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-dev
```

### ROS 2 Integration
```bash
# Install RealSense ROS 2 wrapper
sudo apt install ros-humble-realsense2-camera
```

### Testing the Setup
```bash
# Test RealSense camera
realsense-viewer

# Launch RealSense ROS 2 node
ros2 launch realsense2_camera rs_launch.py
```

## Connecting Hardware Components

### Power Management
- Ensure the Jetson Orin is connected to a reliable 19V power supply
- Use a power distribution board for multiple components
- Implement proper power sequencing to avoid inrush currents

### Communication Interfaces
- USB 3.0 connection for RealSense camera
- GPIO pins for basic sensors and actuators
- Ethernet for high-bandwidth data transfer
- UART for communication with motor controllers

## Troubleshooting Common Issues

### RealSense Camera Not Detected
1. Check USB 3.0 connection
2. Verify kernel modules: `lsusb | grep Intel`
3. Update firmware: `sudo realsense-update-firmware`

### Jetson Orin Overheating
1. Ensure proper heatsink installation
2. Check fan operation
3. Monitor temperature: `sudo tegrastats`

### ROS 2 Node Communication Issues
1. Verify network configuration
2. Check ROS domain ID: `echo $ROS_DOMAIN_ID`
3. Ensure nodes are on the same network segment