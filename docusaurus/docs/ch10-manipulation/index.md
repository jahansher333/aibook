---
id: ch10
title: "Manipulation: Grasping, Force Control, and MoveIt2"
sidebar_position: 10
week: 10
objectives:
  - "Understand grasp quality metrics (form closure, force closure, GWS)"
  - "Implement parallel-jaw and multi-finger grasping strategies"
  - "Use force/torque sensors for compliant manipulation (admittance control)"
  - "Plan collision-free trajectories with MoveIt2 for pick-and-place tasks"
  - "Integrate vision-based grasp pose estimation (6D pose from point clouds)"
tags: [manipulation, grasping, force-control, moveit2, pick-and-place]
description: "Master robotic manipulation: grasp planning, force control, MoveIt2 motion planning, and vision-based pose estimation for pick-and-place."
---

import MCQ from '@site/src/components/MCQ';
import PersonalizeButton from '@site/src/components/PersonalizeButton/PersonalizeButton';
import UrduTranslationButton from '@site/src/components/UrduTranslationButton';

<PersonalizeButton chapterId="ch10" chapterContent="Manipulation" />

<UrduTranslationButton chapterId="ch10" />

# Manipulation: Grasping, Force Control, and MoveIt2

## Learning Objectives

1. **Understand** grasp quality metrics (form closure, force closure, GWS)
2. **Implement** parallel-jaw and multi-finger grasping strategies
3. **Use** force/torque sensors for compliant manipulation (admittance control)
4. **Plan** collision-free trajectories with MoveIt2 for pick-and-place tasks
5. **Integrate** vision-based grasp pose estimation (6D pose from point clouds)

---

## Theory

### 10.1 Grasp Quality and Stability

**TODO**: Form closure (geometric constraint), force closure (contact forces), Grasp Wrench Space (GWS), epsilon metric.

### 10.2 Force Control and Compliant Manipulation

**TODO**: Impedance control (position control with compliance), admittance control (force control), hybrid position/force control.

### 10.3 MoveIt2 for Pick-and-Place

**TODO**: Planning scene, collision objects, cartesian path planning, gripper action server integration.

---

## Hands-on Lab

### Lab 10.1: Parallel-Jaw Gripper Grasp in Isaac Sim

**TODO**: Simulate Robotiq 2F-85 gripper, grasp cube, evaluate contact forces, adjust grip strength (10N → 50N).

### Lab 10.2: Force-Controlled Insertion Task

**TODO**: Insert peg into hole using force feedback (max 10N insertion force), demonstrate compliance (peg tilts without breaking).

### Lab 10.3: Vision-Based Pick-and-Place with MoveIt2

**TODO**: Use PCL (Point Cloud Library) to segment objects, estimate 6D pose (ICP registration), plan grasp approach with MoveIt2, execute in Isaac Sim.

---

## Assessment

<MCQ
  id="ch10-mcq-01"
  question="What is 'form closure' in robotic grasping?"
  options={[
    "A grasp where contact forces alone (friction, normal forces) prevent object motion",
    "A grasp where contact geometry alone (finger positions) prevents object motion, regardless of friction",
    "A grasp that minimizes the number of fingers used",
    "A grasp that requires at least 4 contact points"
  ]}
  correctIndex={1}
  explanation="Correct! Form closure means the object is immobilized by contact geometry (e.g., fingers surround the object), even with zero friction. Force closure relaxes this, requiring friction to prevent slipping. See Section 10.1 grasp quality metrics."
  difficulty="hard"
/>

<MCQ
  id="ch10-mcq-02"
  question="In impedance control, what does 'stiffness' represent?"
  options={[
    "The maximum force the robot can exert",
    "The resistance to deviation from the desired position (high stiffness = rigid, low stiffness = compliant)",
    "The speed at which the gripper closes",
    "The friction coefficient between gripper and object"
  ]}
  correctIndex={1}
  explanation="Correct! Stiffness (K) in impedance control defines how much force is generated per unit displacement: F = K * (x_desired - x_actual). High K makes the robot resist position errors (stiff), low K allows compliance (soft). See Section 10.2 impedance control."
  difficulty="medium"
/>

<MCQ
  id="ch10-mcq-03"
  question="Why is force control necessary for peg-in-hole insertion tasks?"
  options={[
    "To increase insertion speed beyond 10 cm/s",
    "To prevent excessive forces that could damage the peg, hole, or robot, and to accommodate alignment errors via compliance",
    "To reduce computational cost by skipping vision-based pose estimation",
    "To eliminate the need for accurate peg position measurements"
  ]}
  correctIndex={1}
  explanation="Correct! Peg-in-hole requires tight tolerances (0.1mm clearance). Pure position control would jam the peg if misaligned. Force control (max 10N threshold) allows the peg to slide/rotate slightly, accommodating small errors without breaking. See Lab 10.2 force-controlled insertion."
  difficulty="medium"
/>

<MCQ
  id="ch10-mcq-04"
  question="What does MoveIt2's 'planning scene' contain?"
  options={[
    "Only the robot's URDF model",
    "The robot, collision objects (obstacles, table, walls), and allowed collision matrix (ACM)",
    "The global path plan and local trajectory",
    "The camera images used for object detection"
  ]}
  correctIndex={1}
  explanation="Correct! The planning scene includes the robot model (URDF, current joint states), collision objects (boxes, meshes representing obstacles), and ACM (defines which collisions are allowed, e.g., gripper-object during grasp). MoveIt2 uses this to compute collision-free paths. See Section 10.3 MoveIt2 planning scene."
  difficulty="medium"
/>

<MCQ
  id="ch10-mcq-05"
  question="What algorithm does PCL (Point Cloud Library) typically use for 6D pose estimation of objects from depth camera data?"
  options={[
    "YOLO (object detection)",
    "ICP (Iterative Closest Point) or RANSAC for aligning CAD model to observed point cloud",
    "A* (path planning)",
    "Kalman filter (state estimation)"
  ]}
  correctIndex={1}
  explanation="Correct! ICP iteratively aligns the object's CAD model (point cloud) to the observed depth camera point cloud, minimizing point-to-point distances. RANSAC (Random Sample Consensus) is used for robust fitting (rejects outliers). YOLO detects 2D bounding boxes, not 6D pose. See Lab 10.3 vision-based pick-and-place."
  difficulty="hard"
/>

---

## Summary

In this chapter, you learned:

1. **Grasp quality**: Form closure, force closure, GWS, epsilon metric
2. **Force control**: Impedance/admittance control for compliant manipulation
3. **MoveIt2**: Collision-aware motion planning, planning scene, gripper actions
4. **Vision**: 6D pose estimation from point clouds (ICP, RANSAC), PCL segmentation
5. **Integration**: Pick-and-place pipeline (detect → plan → grasp → place)

**Key Takeaways**:
- Good grasps resist arbitrary wrenches (GWS analysis)
- Force control enables safe interaction with uncertain environments
- MoveIt2 + vision closes the perception-action loop

**Next Steps**: In [Chapter 11: Conversational AI Integration](/docs/ch11-conversational-ai), you'll combine manipulation with GPT-4-based task planning.

---

## Further Reading

- Murray, R. M., Li, Z., & Sastry, S. S. (1994). *A Mathematical Introduction to Robotic Manipulation*. CRC Press.
- [MoveIt2 Pick and Place Tutorial](https://moveit.picknik.ai/main/doc/examples/pick_and_place/pick_and_place.html)

---

**Chapter 10 Complete** | Next: [Chapter 11: Conversational AI Integration](/docs/ch11-conversational-ai)

---

**TODO**: Expand theory, complete labs, add 3-5 MCQs
