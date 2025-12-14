---
sidebar_position: 2
---

# Gazebo Physics Simulation and URDF for Humanoids

Unified Robot Description Format (URDF) is an XML-based format used to describe robotic systems in ROS. It defines the physical and visual properties of a robot, including links, joints, and their relationships.

## What is URDF?

URDF (Unified Robot Description Format) is the standard format for representing robot models in ROS. It describes the robot's structure, kinematics, and visual appearance using XML. A URDF file contains information about:

- Links: Rigid parts of the robot (e.g., chassis, arms, wheels)
- Joints: Connections between links (e.g., rotational, prismatic)
- Visual: How the robot appears in simulation
- Collision: How the robot interacts with the environment
- Inertial: Physical properties for dynamics simulation

## Basic URDF Structure

A basic URDF file starts with a robot tag and contains links and joints:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Wheel links -->
  <link name="wheel_front_left">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint connecting wheel to base -->
  <joint name="wheel_front_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_front_left"/>
    <origin xyz="0.3 0.3 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

## Link Properties

Each link in a URDF has several properties:

### Visual Properties
Defines how the link appears in visualization tools:
- `geometry`: Shape (box, cylinder, sphere, mesh)
- `material`: Color and texture information
- `origin`: Position and orientation relative to joint

### Collision Properties
Defines how the link interacts with the physics engine:
- `geometry`: Shape for collision detection
- `origin`: Position and orientation relative to joint

### Inertial Properties
Defines the physical properties for dynamics simulation:
- `mass`: Mass of the link
- `inertia`: Inertia tensor values (ixx, ixy, ixz, iyy, iyz, izz)

## Joint Types

URDF supports several joint types:

### Fixed Joint
A joint that does not move:
```xml
<joint name="fixed_joint" type="fixed">
  <parent link="base_link"/>
  <child link="sensor_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>
```

### Continuous Joint
A rotating joint with unlimited range:
```xml
<joint name="continuous_joint" type="continuous">
  <parent link="base_link"/>
  <child link="rotating_part"/>
  <axis xyz="0 0 1"/>
</joint>
```

### Revolute Joint
A rotating joint with limited range:
```xml
<joint name="revolute_joint" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>
```

### Prismatic Joint
A sliding joint:
```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="slider_link"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="0.5" effort="10" velocity="1"/>
</joint>
```

## Gazebo Integration

To use URDF models in Gazebo, additional Gazebo-specific tags can be added:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>
```

## Robot State Publisher

To visualize the robot in RViz, you need to publish the robot state:

```bash
# Launch the robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(xacro my_robot.urdf.xacro)'
```

## Best Practices

1. **Use Xacro**: Xacro is a macro language that extends URDF, allowing for variables, math, and reuse
2. **Start Simple**: Begin with a basic model and add complexity gradually
3. **Validate**: Use `check_urdf` to validate your URDF file
4. **Test in Simulation**: Always test your URDF in Gazebo before physical implementation
5. **Follow Naming Conventions**: Use consistent and descriptive names for links and joints

## Common URDF Tools

- `check_urdf`: Validates URDF syntax
- `urdf_to_graphiz`: Creates a visual representation of the kinematic tree
- `xacro`: Processes Xacro files to URDF

URDF is fundamental to robotics development in ROS, providing the bridge between robot design and simulation environments like Gazebo.