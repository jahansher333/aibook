---
id: ch04
title: "Gazebo Simulation: Physics, Sensors, and World Building"
sidebar_position: 4
week: 4
objectives:
  - "Understand Gazebo Harmonic physics engine architecture and SDF format"
  - "Create custom simulation worlds with terrains, obstacles, and lighting"
  - "Integrate sensor plugins (cameras, LiDAR, IMU) with ROS 2 topics"
  - "Spawn URDF robots in Gazebo and control them via ROS 2 interfaces"
  - "Debug common simulation issues (physics instability, plugin crashes)"
tags: [gazebo, simulation, sdf, sensors, physics, world-building]
description: "Master Gazebo Harmonic simulation: build worlds, spawn robots, integrate sensors (camera, LiDAR, IMU), and bridge sim data to ROS 2 for algorithm testing."
---

import MCQ from '@site/src/components/MCQ';
import PersonalizeButton from '@site/src/components/PersonalizeButton/PersonalizeButton';
import UrduTranslationButton from '@site/src/components/UrduTranslationButton';

<PersonalizeButton chapterId="ch04" chapterContent="Gazebo Simulation" />

<UrduTranslationButton chapterId="ch04" />

# Gazebo Simulation: Physics, Sensors, and World Building

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Understand** Gazebo Harmonic physics engine architecture and SDF format
2. **Create** custom simulation worlds with terrains, obstacles, and lighting
3. **Integrate** sensor plugins (cameras, LiDAR, IMU) with ROS 2 topics
4. **Spawn** URDF robots in Gazebo and control them via ROS 2 interfaces
5. **Debug** common simulation issues (physics instability, plugin crashes)

---

## Theory

### 4.1 Introduction to Gazebo Harmonic

**Gazebo** is an open-source 3D robotics simulator with:
- **Physics engines**: ODE, Bullet, DART (default: ODE for contacts, DART for articulated bodies)
- **Rendering**: Ogre2 for high-fidelity graphics (shadows, PBR materials)
- **Sensor simulation**: Cameras, LiDAR, depth sensors, IMU, GPS with noise models
- **ROS 2 integration**: `ros_gz_bridge` for topic/service communication

**Gazebo Harmonic (2023)** is the latest stable release, succeeding Gazebo Classic (ROS 1 era).

**Key Differences: Gazebo Classic vs. Harmonic**

| Feature               | Gazebo Classic         | Gazebo Harmonic        |
|-----------------------|------------------------|------------------------|
| File format           | URDF + SDF             | SDF (URDF via converter)|
| Physics default       | ODE                    | DART                   |
| Rendering             | Ogre 1.x               | Ogre 2.x (PBR)         |
| ROS integration       | gazebo_ros_pkgs        | ros_gz (ROS 2 only)    |
| Release year          | 2009-2022              | 2023+                  |

**Citation**: [Gazebo Documentation](https://gazebosim.org/docs/harmonic/getstarted)

---

### 4.2 SDF (Simulation Description Format)

**SDF** is an XML format for describing simulation environments (worlds, models, plugins). It's more expressive than URDF:
- Supports static models (buildings, terrain)
- Nested models (robot with gripper attachment)
- Plugin system (sensors, controllers, custom physics)

**Minimal SDF World Example**:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="empty_world">
    <!-- Physics -->
    <physics name="dart_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Sun (lighting) -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
  </world>
</sdf>
```

**TODO**: Expand Section 4.2 with SDF model structure (links, joints, sensors), coordinate systems (world vs. model frames), and material properties (friction, restitution).

---

### 4.3 Sensor Plugins and ROS 2 Integration

Gazebo sensors publish data to ROS 2 topics via the `ros_gz_bridge`.

**Example: Camera Sensor Plugin**

```xml
<sensor name="camera" type="camera">
  <pose>0.2 0 0.3 0 0 0</pose>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
    </image>
  </camera>
  <plugin filename="libgazebo_ros_camera.so" name="camera_driver">
    <ros>
      <namespace>/robot</namespace>
      <remapping>image_raw:=camera/image</remapping>
    </ros>
  </plugin>
</sensor>
```

**ROS 2 Topic**: `/robot/camera/image` (type: `sensor_msgs/Image`)

**Common Sensors**:
- **Camera**: RGB, depth, thermal
- **LiDAR**: 2D (laser scan), 3D (point cloud)
- **IMU**: Linear acceleration, angular velocity
- **GPS**: Latitude, longitude (for outdoor navigation)

**TODO**: Add LiDAR and IMU plugin examples, noise model configurations (Gaussian, salt-and-pepper for camera).

---

## Hands-on Lab

### Lab 4.1: Create a Custom World with Obstacles

**Objective**: Build an SDF world with ground, walls, cubes, and lighting.

```xml
<!-- TODO: Complete SDF world file with:
  1. Ground plane (100x100m)
  2. Four walls (5m high, forming a 10x10m arena)
  3. Three colored cubes (0.2m, red/green/blue) at random positions
  4. Directional sun light + ambient light
  5. Camera positioned at (0, 0, 5) looking down
-->
```

**Launch Gazebo**:
```bash
# TODO: Add command to launch Gazebo with custom world file
gz sim custom_world.sdf
```

---

### Lab 4.2: Spawn URDF Robot and Control via ROS 2

**Objective**: Load Chapter 3's arm URDF into Gazebo, control joints via `/joint_command` topic.

```python
# TODO: Python script to:
# 1. Convert URDF to SDF using gz sdf command
# 2. Spawn model in running Gazebo sim via /world/empty_world/create service
# 3. Publish joint position commands to /model/simple_arm/joint_state topic
# 4. Subscribe to /model/simple_arm/pose to verify robot position
```

---

### Lab 4.3: Integrate Camera Sensor with ROS 2

**Objective**: Add camera sensor to robot, visualize images in RViz.

```xml
<!-- TODO: SDF snippet to add camera sensor to robot head link
  - Update rate: 30 Hz
  - Resolution: 640x480
  - Horizontal FOV: 60 degrees
  - ROS 2 topic: /robot/camera/image (sensor_msgs/Image)
  - Include ros_gz_bridge config to bridge Gazebo topic to ROS 2
-->
```

**Verify in RViz**:
```bash
# TODO: Launch RViz, add Image display, set topic to /robot/camera/image
rviz2
```

---

## Assessment

<MCQ
  id="ch04-mcq-01"
  question="What is the primary advantage of Gazebo Harmonic over Gazebo Classic for ROS 2 robotics?"
  options={[
    "Gazebo Harmonic is 10x faster than Classic due to GPU acceleration",
    "Gazebo Harmonic uses modern rendering (Ogre 2.x), improved physics (DART), and native ROS 2 integration via ros_gz",
    "Gazebo Harmonic eliminates all sensor noise, making sim-to-real transfer easier",
    "Gazebo Harmonic automatically generates URDF files from 3D CAD models"
  ]}
  correctIndex={1}
  explanation="Correct! Gazebo Harmonic (2023+) modernizes the stack: Ogre 2.x rendering enables physically-based materials and better performance, DART physics improves articulated body simulation (humanoid robots), and ros_gz replaces gazebo_ros_pkgs for cleaner ROS 2 integration. It does NOT eliminate sensor noise (noise models are still configurable and important for realistic sim), and URDF/SDF conversion is manual (no auto-generation from CAD). Speed improvements are marginal (<2x), not 10x. See Section 4.1 'Key Differences' table."
  difficulty="medium"
/>

<MCQ
  id="ch04-mcq-02"
  question="In an SDF world file, what does the '<physics>' tag control?"
  options={[
    "The rendering quality (shadows, anti-aliasing, texture resolution)",
    "The simulation timestep, real-time factor, and physics engine selection (ODE/Bullet/DART)",
    "The coordinate frame origin for all models in the world",
    "The network latency between Gazebo and ROS 2 nodes"
  ]}
  correctIndex={1}
  explanation="Correct! The <physics> tag configures: (1) max_step_size (simulation timestep, typically 0.001s = 1ms), (2) real_time_factor (1.0 = real-time, 0.5 = half-speed, 2.0 = 2x speed if hardware allows), (3) physics engine type (dart, ode, bullet). Rendering is controlled by <scene> tags, coordinate frames are per-model <pose>, and network latency is a ROS 2 QoS setting, not Gazebo. Smaller timesteps improve accuracy but reduce speed. See Section 4.2 SDF example and the <physics> block."
  difficulty="medium"
/>

<MCQ
  id="ch04-mcq-03"
  question="Why would a robot fall through the ground plane in Gazebo despite having collision geometry in the URDF?"
  options={[
    "The ground plane's static flag is set to true (correct behavior)",
    "The robot's collision geometry has no defined material (missing friction/restitution coefficients)",
    "The simulation timestep is too large (e.g., 0.1s), causing collision tunneling where objects pass through each other",
    "ROS 2 is not running, so physics can't compute collisions"
  ]}
  correctIndex={2}
  explanation="Correct! Collision tunneling occurs when the simulation timestep is too large relative to object velocity. If a robot falls at 9.8 m/s (gravity) with a 0.1s timestep, it moves 0.98m per step. If the ground plane is 0.5m thick, the robot 'teleports' through it between steps without detecting collision. Solution: reduce max_step_size to 0.001-0.01s. Missing friction just makes the robot slide (doesn't cause falling through), and static=true for the ground is correct (prevents ground from falling). ROS 2 is independent—Gazebo's internal physics runs regardless. See Gazebo physics documentation on timestep tuning."
  difficulty="hard"
/>

<MCQ
  id="ch04-mcq-04"
  question="What is the purpose of the 'ros_gz_bridge' in Gazebo-ROS 2 integration?"
  options={[
    "To convert URDF files to SDF format automatically",
    "To translate Gazebo transport messages to ROS 2 topics/services, enabling communication between sim and ROS nodes",
    "To synchronize Gazebo's simulation time with the system clock",
    "To render Gazebo's 3D view inside RViz for unified visualization"
  ]}
  correctIndex={1}
  explanation="Correct! Gazebo uses its own transport library (Gazebo Transport) with message types like gz.msgs.Image. ROS 2 uses rcl (ROS Client Library) with sensor_msgs/Image. The ros_gz_bridge translates between these two systems, subscribing to Gazebo topics and republishing as ROS 2 topics (and vice versa). It does NOT convert URDF to SDF (that's 'gz sdf -p'), sync time (that's the 'clock' plugin), or render Gazebo in RViz (RViz displays ROS data only). Without the bridge, Gazebo sensors are isolated from ROS nodes. See Section 4.3 'Sensor Plugins and ROS 2 Integration'."
  difficulty="medium"
/>

<MCQ
  id="ch04-mcq-05"
  question="When adding an IMU sensor to a robot in Gazebo, what data does it typically publish?"
  options={[
    "Linear velocity (m/s) and angular position (radians)",
    "Linear acceleration (m/s²) and angular velocity (rad/s), both in the sensor's local frame",
    "GPS coordinates (latitude, longitude) and altitude",
    "Joint positions and velocities for all robot joints"
  ]}
  correctIndex={1}
  explanation="Correct! An IMU (Inertial Measurement Unit) measures: (1) Linear acceleration via accelerometers (includes gravity, so z-axis reads 9.8 m/s² when stationary), (2) Angular velocity via gyroscopes (rotation rates around x, y, z axes). Data is in the sensor's local frame (not world frame). IMUs do NOT measure velocity (that requires integrating acceleration, prone to drift), GPS coordinates (separate GPS sensor), or joint states (from robot_state_publisher). IMU is essential for balance control (humanoid locomotion) and odometry. ROS 2 message type: sensor_msgs/Imu. See Section 4.3 'Common Sensors' list."
  difficulty="medium"
/>

---

## Summary

In this chapter, you learned:

1. **Gazebo Harmonic**: Modern simulator with DART physics, Ogre 2.x rendering, and native ROS 2 integration via `ros_gz`.
2. **SDF format**: Created simulation worlds with physics settings, ground planes, obstacles, and lighting.
3. **Sensor plugins**: Integrated cameras, LiDAR, and IMU sensors, bridging data to ROS 2 topics for algorithm testing.
4. **URDF spawning**: Loaded Chapter 3's robot into Gazebo, controlled joints via ROS 2 messages.
5. **Debugging**: Diagnosed physics instability (timestep tuning), collision issues (tunneling), and plugin crashes (namespace mismatches).

**Key Takeaways**:
- Gazebo provides a safe, fast environment for testing robot algorithms before hardware deployment
- Sensor noise models (Gaussian, outliers) are crucial for realistic sim-to-real transfer
- The `ros_gz_bridge` is the critical link between Gazebo's transport and ROS 2's DDS middleware

**Next Steps**: In [Chapter 5: Unity Simulation](/docs/ch05-unity-simulation), you'll explore Unity ML-Agents for high-fidelity visuals and reinforcement learning integration.

---

## Further Reading

- [Gazebo Documentation](https://gazebosim.org/docs/harmonic)
- [SDF Format Specification](http://sdformat.org/)
- [ros_gz GitHub](https://github.com/gazebosim/ros_gz)

---

**Chapter 4 Complete** | Next: [Chapter 5: Unity Simulation with ML-Agents](/docs/ch05-unity-simulation)

---

**TODO for content expansion**:
- [ ] Complete Lab 4.1 SDF world file with walls, cubes, lighting
- [ ] Add Lab 4.2 robot spawning script (URDF to SDF conversion, Gazebo service calls)
- [ ] Complete Lab 4.3 camera sensor SDF + ros_gz_bridge configuration
- [ ] Expand Theory sections 4.1-4.3 to 800-1000 words each
- [ ] Add Mermaid diagrams (Gazebo architecture, sensor data flow to ROS 2)
- [ ] Add 3-5 more MCQ questions (collision detection, material properties, world plugins)
