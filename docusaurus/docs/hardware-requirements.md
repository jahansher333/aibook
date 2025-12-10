---
id: hardware-requirements
title: Hardware Requirements
sidebar_position: 15
---

# Hardware Requirements

This guide helps you choose the right hardware tier for your learning journey in Physical AI and humanoid robotics. We offer three distinct paths to fit different budgets, learning styles, and research needs.

## Quick Comparison

| Feature | Economy | Proxy | Cloud OpEx |
|---------|---------|-------|-----------|
| **Total Cost** | $700 (one-time) | $3,850 (one-time) | $205/quarter (~$68/mo) |
| **Best For** | Solo learners, students | Research labs, institutions | Cloud-first, remote learning |
| **Compute** | Jetson Orin Nano | Jetson Xavier NX + Go2 | AWS EC2 T4 GPU |
| **Physical Robot** | None | Unitree Go2 | None |
| **Vision** | 1x RealSense D435i | Inbuilt + upgradable | Cloud-based |
| **Learning Curve** | Beginner-friendly | Moderate-advanced | Intermediate |
| **Real-World Testing** | Simulation-heavy | ✅ Outdoor capable | Cloud simulations |
| **Setup Time** | ~2 hours | ~1 day | ~30 minutes |

## Tier 1: Economy (~$700)

### Overview

The Economy tier provides a complete, self-contained entry point to Physical AI. With NVIDIA's Jetson Orin Nano and Intel RealSense D435i, you'll have everything needed to learn ROS 2, computer vision, and AI fundamentals on real embedded hardware.

**Total Investment: $700 USD (one-time)**

### Component Breakdown

| Component | Cost | Specifications | Vendor |
|-----------|------|----------------|--------|
| **Jetson Orin Nano Developer Kit** | $249 | 8-core ARM, 1024 CUDA cores, 8GB LPDDR5, 100 TOPS | [NVIDIA](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) |
| **Intel RealSense D435i RGB-D** | $349 | 1280x720 RGB, stereo depth, built-in IMU, 90fps | [Intel](https://www.intelrealsense.com/depth-camera-d435i/) |
| **Accessories & Cables** | $102 | USB cables, mounts, heatsinks, 128GB µSD, 65W PSU | Various |

### Use Cases

- ✅ Computer vision with OpenCV and ROS 2
- ✅ ROS 2 fundamentals on real hardware (nodes, topics, services)
- ✅ Running lightweight AI models (ResNet, MobileNet, CLIP inference)
- ✅ Sim-to-real transfer exercises with Isaac Sim
- ✅ Educational robotics projects and capstone work

### Advantages

- **Most affordable**: Entry-level cost fits student budgets
- **Production-ready**: Full NVIDIA CUDA ecosystem
- **Compact**: 15W power consumption, fits in backpack
- **Well-supported**: Extensive community tutorials and ROS packages
- **Flexible**: Supports Python, C++, and containerized workloads

### Limitations

- **Compute-constrained**: Not suitable for training large VLA models
- **No locomotion**: Limited to stationary vision tasks without external robotics
- **Single modality**: One RGB-D camera only
- **Power management**: Requires external battery for mobile applications

### Getting Started

```bash
# Flash JetPack OS to microSD card (6.0+ recommended)
wget https://developer.nvidia.com/jetpack

# Connect RealSense D435i via USB 3.1
sudo apt install -y librealsense2-dev python3-librealsense2

# Verify installation
realsense-viewer

# Clone this textbook's examples
git clone https://github.com/physical-ai/textbook.git
cd textbook/ch02-ros2-fundamentals
```

**Estimated Setup Time**: 2-3 hours

---

## Tier 2: Proxy (Embodied) (~$3,850)

### Overview

The Proxy tier gives you a complete embodied AI platform: the Unitree Go2 quadruped robot with onboard Jetson Xavier NX compute. This is the choice for research labs and institutions seeking real-world robotics deployment and whole-body control research.

**Total Investment: $3,850 USD (one-time)**

### Component Breakdown

| Component | Cost | Specifications | Vendor |
|-----------|------|----------------|--------|
| **Unitree Go2 Quadruped** | $3,000 | 12 DOF, Jetson Xavier NX, LiDAR, cameras, 45-min battery | [Unitree](https://www.unitreerobotics.com/products/go2) |
| **Jetson Xavier NX Expansion** | $500 | 8-core ARM, 384 CUDA cores, 8GB LPDDR4x, 21 TOPS | [NVIDIA](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-xavier-nx/) |
| **Sensors & Accessories** | $350 | RGB-D upgrades, IMU kit, battery mgmt, case, spare parts | Various |

### Use Cases

- ✅ Quadruped locomotion control (gait generation, stability)
- ✅ Whole-body control with proprioceptive feedback
- ✅ Multi-modal learning (vision + proprioception + contact)
- ✅ Outdoor autonomous navigation (1.5 m/s max speed)
- ✅ Imitation learning from human demonstrations
- ✅ Research publications and conference demonstrations

### Advantages

- **Complete embodied platform**: All sensors and actuators integrated
- **Outdoor-ready**: Weatherproof design, field-deployable
- **Real proprioception**: IMU, joint encoders, and contact sensing
- **Active research**: Supported by robotics community (papers, benchmarks)
- **Scalable learning**: Run complex policies on onboard Jetson

### Limitations

- **Significant cost**: Requires substantial budget commitment
- **Maintenance needed**: Motors, batteries, and sensors require care
- **Slower training**: Jetson Xavier NX slower than desktop GPUs
- **Battery-dependent**: ~45-minute runtime requires charging management
- **Steep learning curve**: Robotics debugging skills essential

### Getting Started

```bash
# Initialize Go2 with Unitree SDK
git clone https://github.com/unitreerobotics/unitree_ros2.git
cd unitree_ros2 && colcon build

# Calibrate IMU and joint encoders
roslaunch go2_base init_calibration.launch

# Test basic locomotion
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'

# Run a trajectory
ros2 run go2_control walking_gait.py
```

**Estimated Setup Time**: 1-2 days (including calibration)

---

## Tier 3: Cloud OpEx (~$205/quarter)

### Overview

The Cloud OpEx tier provides unlimited GPU compute on AWS without hardware investment. Ideal for AI training, simulation at scale, and remote collaboration. Pay only for what you use—no upfront capital costs.

**Recurring Cost: $205/quarter (~$68/month)**

### Component Breakdown

| Service | Cost/Quarter | Specifications | Vendor |
|---------|--------------|----------------|--------|
| **EC2 g4dn.xlarge** | $192 | 1x T4 GPU (16GB), 4 vCPU, 16GB RAM | [AWS](https://aws.amazon.com/ec2/instance-types/g4/) |
| **EBS Storage (gp3)** | $10 | 200 GB storage, 1000 IOPS | [AWS](https://aws.amazon.com/ebs/) |
| **Data Transfer** | $3 | Outbound internet, inter-region | [AWS](https://aws.amazon.com/ec2/pricing/on-demand/) |

### Use Cases

- ✅ Training large Vision-Language-Action (VLA) models
- ✅ Batch simulation and trajectory evaluation
- ✅ Running Isaac Sim in cloud (GPU-accelerated rendering)
- ✅ Collaborative research with distributed teams
- ✅ Prototyping before deploying to physical robots
- ✅ Continuous integration for robotics pipelines

### Advantages

- **Zero upfront cost**: No hardware to purchase or maintain
- **Infinite scalability**: Spin up multiple GPUs on-demand
- **Pay-as-you-go**: Only pay for active usage
- **Latest GPUs**: Access cutting-edge T4, V100, A100 accelerators
- **Professional support**: AWS SLA, monitoring, and backup

### Limitations

- **Recurring expenses**: Costs accumulate over time
- **Network latency**: Unsuitable for real-time robot control
- **Data egress charges**: Moving large datasets can be expensive
- **Less hands-on**: Limited to simulation and training, no physical interaction
- **Cloud complexity**: Requires AWS account, VPC, security setup

### Getting Started

```bash
# Launch EC2 instance with NVIDIA AMI
aws ec2 run-instances \
  --image-id ami-0c55b159cbfafe1f0 \
  --instance-type g4dn.xlarge \
  --region us-west-2

# SSH into instance
ssh -i your-key.pem ubuntu@<instance-ip>

# Verify GPU
nvidia-smi

# Install ROS 2 and dependencies
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo apt install -y ros-humble-desktop

# Clone textbook examples
git clone https://github.com/physical-ai/textbook.git
cd textbook && pip install -r requirements.txt
```

**Estimated Setup Time**: 30 minutes

---

## Decision Guide

### Choose Economy if:

- You're a **student or solo learner** with a limited budget
- You want to master **ROS 2 fundamentals** on real embedded hardware
- You're interested in **computer vision** and **AI inference**
- You plan to **sim-to-real transfer** later with a physical robot
- You value **independence** and **learning by doing**

### Choose Proxy if:

- You're a **research lab or institution** with dedicated funding
- You need to study **legged locomotion** and **whole-body control**
- You want to conduct **real-world outdoor experiments**
- You plan to **publish research** with physical robot results
- You have **technical staff** for robot maintenance

### Choose Cloud if:

- You're a **remote learner** or **distributed research team**
- You focus on **AI training and simulation** rather than hardware
- You need **on-demand GPU scaling** for batch jobs
- You want to **minimize upfront capital costs**
- You prefer **managed infrastructure** over hardware maintenance

---

## Hybrid Approach

Many learners combine tiers for maximum flexibility:

| Path | Description |
|------|-------------|
| **Economy → Cloud** | Learn fundamentals on Jetson, then scale AI training to AWS |
| **Cloud → Proxy** | Prototype in simulation, validate with physical Go2 |
| **Economy + Cloud** | Run local ROS 2 nodes, offload vision inference to cloud |

---

## Price Notes & Disclaimers

- **Prices accurate as of**: December 2025
- **Subject to change**: Contact vendors for current pricing
- **Regional variation**: Cloud costs vary by AWS region; hardware availability varies by country
- **Bulk discounts**: Institutions may qualify for academic pricing from NVIDIA and AWS
- **Shipping**: International shipping may add 10-20% to hardware costs
- **Taxes**: Prices exclude local sales tax or VAT

### Verified Sources

- [NVIDIA Jetson Developer Shop](https://www.nvidia.com/)
- [Intel RealSense Official Store](https://www.intelrealsense.com/)
- [Unitree Robotics Shop](https://www.unitreerobotics.com/)
- [AWS EC2 Pricing Calculator](https://aws.amazon.com/ec2/pricing/on-demand/)

---

## Technical Specifications Reference

### Jetson Orin Nano (Economy Core)

```
Architecture: ARM64 (AArch64)
CPU: 8-core ARM Cortex-A78AE @ 3.5 GHz
GPU: NVIDIA Ampere, 1024 CUDA cores
Memory: 8 GB LPDDR5
Storage: microSD (user-provided)
Power: 5W-15W (variable)
Peak AI: 100 TOPS (int8)
```

### Jetson Xavier NX (Proxy Expansion)

```
Architecture: ARM64 (AArch64)
CPU: 8-core ARM Cortex-A57 @ 2.0 GHz
GPU: NVIDIA Volta, 384 CUDA cores
Memory: 8 GB LPDDR4x
Storage: microSD or eMMC
Power: 10W-15W (variable)
Peak AI: 21 TOPS (int8)
```

### Intel RealSense D435i

```
RGB Sensor: 1280 x 720 @ 90 fps
Depth Sensor: Stereo passive IR (OmniVision OV9782)
Depth Range: 0.105m - 1280m
IMU: 6-axis (accel + gyro)
Connectivity: USB 3.1
Streaming: H.264/H.265 video
```

### AWS EC2 g4dn.xlarge

```
vCPU: 4 (Intel Cascade Lake)
Memory: 16 GB DDR4
GPU: 1x NVIDIA T4 (16 GB VRAM)
Network: Up to 25 Gbps
Storage: EBS-optimized (1000 Mbps)
On-Demand: $0.526/hour (US West 2)
```

---

## Next Steps

1. **Choose your tier** based on your budget and learning goals
2. **Set up your environment** following the Getting Started guides above
3. **Begin Chapter 1**: [Introduction to Physical AI](/docs/ch01-physical-ai-intro)
4. **Join the community**: [Physical AI Slack workspace](https://slack.physical-ai.org)
5. **Ask questions**: [GitHub Discussions](https://github.com/physical-ai/textbook/discussions)

---

**Last Updated**: December 9, 2025
**Maintainers**: Physical AI Textbook Team
**License**: CC-BY-SA 4.0 (Content), MIT (Code)
