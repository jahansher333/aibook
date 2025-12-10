---
id: ch12
title: "Hardware Integration: Deploying to Jetson Orin Nano"
sidebar_position: 12
week: 12
objectives:
  - "Setup Jetson Orin Nano with ROS 2 Humble and Isaac ROS packages"
  - "Integrate Intel RealSense D435i for depth sensing and visual odometry"
  - "Deploy trained models (ONNX) for edge inference with TensorRT optimization"
  - "Implement real-time monitoring (CPU, GPU, thermal throttling) with tegrastats"
  - "Debug common hardware issues (USB bandwidth, power delivery, sensor calibration)"
tags: [jetson, hardware, realsense, tensorrt, deployment, embedded]
description: "Deploy Physical AI systems to Jetson Orin Nano: ROS 2 setup, RealSense integration, ONNX/TensorRT inference, and hardware debugging."
---

import MCQ from '@site/src/components/MCQ';
import PersonalizeButton from '@site/src/components/PersonalizeButton/PersonalizeButton';
import UrduTranslationButton from '@site/src/components/UrduTranslationButton';

<PersonalizeButton chapterId="ch12" chapterContent="Hardware Integration" />

<UrduTranslationButton chapterId="ch12" />

# Hardware Integration: Deploying to Jetson Orin Nano

## Learning Objectives

1. **Setup** Jetson Orin Nano with ROS 2 Humble and Isaac ROS packages
2. **Integrate** Intel RealSense D435i for depth sensing and visual odometry
3. **Deploy** trained models (ONNX) for edge inference with TensorRT optimization
4. **Implement** real-time monitoring (CPU, GPU, thermal throttling) with tegrastats
5. **Debug** common hardware issues (USB bandwidth, power delivery, sensor calibration)

---

## Theory

### 12.1 Jetson Orin Nano Hardware Architecture

**Specs**:
- **CPU**: 6-core ARM Cortex-A78AE @ 1.5 GHz
- **GPU**: 1024-core NVIDIA Ampere (40 TOPS INT8)
- **RAM**: 8GB LPDDR5 (unified memory, shared CPU/GPU)
- **Storage**: microSD (recommend 128GB A2 UHS-I for OS + models)
- **Power**: 15W typical, 25W peak (requires 5V 4A barrel jack or USB-C PD)

**TODO**: Expand with power mode profiles (5W, 10W, 15W, 25W), clock frequency tuning, thermal design considerations.

---

### 12.2 Intel RealSense D435i Integration

**Sensor Specs**:
- **Depth**: Stereo IR cameras (1280x720 @ 30 FPS, range: 0.3-10m)
- **RGB**: 1920x1080 @ 30 FPS
- **IMU**: Accel + Gyro (250 Hz, for visual-inertial odometry)

**ROS 2 Integration**:
```bash
# Install RealSense ROS 2 package
sudo apt install ros-humble-realsense2-camera

# Launch camera node
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30
```

**TODO**: Add camera calibration steps (intrinsics, extrinsics), depth accuracy analysis, USB 3.0 bandwidth requirements (3-5 Gbps).

---

### 12.3 ONNX to TensorRT Conversion for Edge Inference

**Workflow**:
1. Train model in Unity/PyTorch → Export ONNX
2. Convert ONNX to TensorRT engine (optimized for Jetson GPU)
3. Run inference with `trtexec` or Python API

```bash
# Convert ONNX to TensorRT FP16 (half-precision for 2x speedup)
/usr/src/tensorrt/bin/trtexec --onnx=model.onnx --saveEngine=model_fp16.trt --fp16

# Benchmark inference
/usr/src/tensorrt/bin/trtexec --loadEngine=model_fp16.trt --batch=1
```

**TODO**: Add quantization (INT8 calibration), layer profiling, memory optimization strategies.

---

## Hands-on Lab

### Lab 12.1: Jetson Setup and ROS 2 Installation

**TODO**: Flash JetPack 6.0, install ROS 2 Humble via apt, configure swap (8GB zram), install Isaac ROS packages.

### Lab 12.2: RealSense Camera Integration

**TODO**: Connect D435i via USB 3.0, launch camera node, visualize depth/RGB in RViz, run visual odometry (T265 alternative: `realsense2_camera + rtabmap_ros`).

### Lab 12.3: Deploy ONNX Model with TensorRT

**TODO**: Convert Unity-trained walker policy (ONNX) to TensorRT FP16, create ROS 2 inference node, measure latency (&lt;50ms target), compare to ONNX Runtime baseline.

---

## Assessment

<MCQ
  id="ch12-mcq-01"
  question="Why does Jetson Orin Nano use unified memory (8GB shared between CPU and GPU)?"
  options={[
    "To reduce manufacturing cost by eliminating dedicated VRAM",
    "To enable zero-copy data transfer between CPU and GPU, reducing latency and memory overhead for robotics workloads",
    "Unified memory is slower than dedicated VRAM but more power-efficient",
    "It allows hot-swapping RAM modules without rebooting"
  ]}
  correctIndex={1}
  explanation="Correct! Unified memory (LPDDR5) allows CPU and GPU to access the same physical memory without copying data. For example, a camera image captured by CPU can be directly accessed by GPU for inference (zero-copy), saving time and RAM. This is crucial for edge devices with limited bandwidth. Desktop GPUs have separate VRAM requiring PCIe transfers. See Section 12.1 Jetson architecture."
  difficulty="medium"
/>

<MCQ
  id="ch12-mcq-02"
  question="What is the advantage of converting an ONNX model to TensorRT format for Jetson deployment?"
  options={[
    "TensorRT engines are human-readable text files, easier to debug",
    "TensorRT optimizes models for specific hardware (layer fusion, precision calibration), achieving 2-5x speedup over ONNX Runtime on Jetson GPUs",
    "TensorRT models are cross-platform and work on CPUs, GPUs, and TPUs",
    "TensorRT automatically improves model accuracy by retraining layers"
  ]}
  correctIndex={1}
  explanation="Correct! TensorRT optimizes ONNX models for Jetson's Ampere GPU: fuses layers (Conv+BatchNorm+ReLU → single kernel), calibrates INT8 quantization, allocates optimal memory. This achieves 2-5x inference speedup vs. ONNX Runtime. TensorRT engines are binary (hardware-specific, NOT cross-platform), and accuracy is preserved (no retraining, only precision reduction if FP16/INT8 used). See Section 12.3 TensorRT conversion."
  difficulty="medium"
/>

<MCQ
  id="ch12-mcq-03"
  question="What does 'tegrastats' tool display on Jetson Orin Nano?"
  options={[
    "ROS 2 topic frequencies and node CPU usage",
    "Real-time CPU/GPU utilization, RAM usage, power consumption, and thermal throttling status",
    "TensorRT inference latency and throughput",
    "Wi-Fi signal strength and network bandwidth"
  ]}
  correctIndex={1}
  explanation="Correct! 'tegrastats' is NVIDIA's system monitor for Jetson, displaying: CPU usage (per core), GPU usage (%), RAM (used/total 8GB), power (W), temperature (°C), and throttling flags (thermal/power limits). Essential for performance tuning. ROS 2 monitoring is via 'ros2 topic hz', TensorRT profiling via 'trtexec --dumpProfile'. See Section 12.4 real-time monitoring (TODO)."
  difficulty="easy"
/>

<MCQ
  id="ch12-mcq-04"
  question="Why might a RealSense D435i camera fail to stream depth images on Jetson despite working on a laptop?"
  options={[
    "Jetson GPUs don't support depth image processing",
    "Insufficient USB 3.0 bandwidth or power delivery (D435i requires USB 3.0 + external power for high FPS)",
    "ROS 2 Humble doesn't support RealSense cameras",
    "Depth cameras only work on x86 CPUs, not ARM"
  ]}
  correctIndex={1}
  explanation="Correct! D435i streams stereo IR (2x 1280x720) + RGB (1920x1080) at 30 FPS = ~3-5 Gbps USB 3.0 bandwidth. Jetson's USB ports share bandwidth, and some cheap cables/hubs don't deliver full USB 3.0 speed. Solution: Use certified USB 3.0 cable, connect to USB 3.2 port (blue), add external 12V power to D435i if available. ROS 2 Humble supports RealSense (ros-humble-realsense2-camera), and ARM CPUs work fine. See Section 12.2 RealSense integration troubleshooting (TODO)."
  difficulty="hard"
/>

<MCQ
  id="ch12-mcq-05"
  question="What is the recommended swap size for Jetson Orin Nano with 8GB RAM when running multiple ROS 2 nodes + inference?"
  options={[
    "No swap needed (8GB RAM is sufficient)",
    "4-8GB zram swap (compressed RAM-based swap for emergency memory)",
    "32GB disk swap (on microSD card for maximum headroom)",
    "Swap is not supported on Jetson devices"
  ]}
  correctIndex={1}
  explanation="Correct! While 8GB RAM is adequate for typical robotics workloads, running ROS 2 nodes + TensorRT inference + RViz can spike memory usage >6GB. 4-8GB zram swap (compressed RAM-based, faster than disk) provides emergency headroom, preventing OOM (Out of Memory) kills. Avoid large disk swap on microSD (slow, wears out flash). Configure via JetPack or `sudo fallocate -l 8G /swapfile`. See Lab 12.1 Jetson setup (TODO)."
  difficulty="medium"
/>

---

## Summary

In this chapter, you learned:

1. **Jetson setup**: Flashed JetPack, installed ROS 2 Humble, configured swap and power modes
2. **RealSense integration**: Streamed depth/RGB/IMU, launched visual odometry
3. **TensorRT deployment**: Converted ONNX → TensorRT FP16, achieved &lt;50ms inference
4. **Monitoring**: Used tegrastats for real-time CPU/GPU/thermal profiling
5. **Debugging**: Resolved USB bandwidth, power delivery, and thermal throttling issues

**Key Takeaways**:
- Jetson's unified memory enables zero-copy robotics pipelines
- TensorRT optimization is critical for real-time edge inference
- Hardware debugging requires understanding bandwidth/power/thermal constraints

**Next Steps**: In [Chapter 13: Capstone Project](/docs/ch13-capstone-project), you'll integrate all skills into a voice-commanded autonomous humanoid.

---

## Further Reading

- [Jetson Orin Nano Developer Guide](https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit)
- [TensorRT Documentation](https://docs.nvidia.com/deeplearning/tensorrt/)

---

**Chapter 12 Complete** | Next: [Chapter 13: Capstone Project](/docs/ch13-capstone-project)

---

**TODO**: Expand theory, complete labs, add 3-5 MCQs
