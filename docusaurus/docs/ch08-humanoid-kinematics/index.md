---
id: ch08
title: "Humanoid Kinematics: Forward/Inverse Kinematics and Jacobian"
sidebar_position: 8
week: 8
objectives:
  - "Derive forward kinematics using Denavit-Hartenberg (DH) parameters"
  - "Implement inverse kinematics (IK) for humanoid arm reaching tasks"
  - "Compute Jacobian matrices for velocity control and singularity analysis"
  - "Use MoveIt2 for motion planning with collision avoidance"
  - "Optimize IK solutions for human-like postures (redundancy resolution)"
tags: [kinematics, inverse-kinematics, jacobian, dh-parameters, moveit2]
description: "Learn forward/inverse kinematics for humanoid robots: DH parameters, Jacobian computation, MoveIt2 integration, and redundancy resolution."
---

import MCQ from '@site/src/components/MCQ';
import PersonalizeButton from '@site/src/components/PersonalizeButton/PersonalizeButton';
import UrduTranslationButton from '@site/src/components/UrduTranslationButton';

<PersonalizeButton chapterId="ch08" chapterContent="Humanoid Kinematics" />

<UrduTranslationButton chapterId="ch08" />

# Humanoid Kinematics: Forward/Inverse Kinematics and Jacobian

## Learning Objectives

1. **Derive** forward kinematics using Denavit-Hartenberg (DH) parameters
2. **Implement** inverse kinematics (IK) for humanoid arm reaching tasks
3. **Compute** Jacobian matrices for velocity control and singularity analysis
4. **Use** MoveIt2 for motion planning with collision avoidance
5. **Optimize** IK solutions for human-like postures (redundancy resolution)

---

## Theory

### 8.1 Forward Kinematics and DH Parameters

**TODO**: Add DH parameter table for 7-DOF humanoid arm, homogeneous transformation matrices, end-effector pose computation.

### 8.2 Inverse Kinematics (Analytical vs. Numerical)

**TODO**: Compare closed-form IK (3-DOF arm) vs. iterative methods (Jacobian pseudoinverse, damped least squares, CCD).

### 8.3 Jacobian and Singularity Analysis

**TODO**: Derive Jacobian matrix, detect singularities (determinant = 0), workspace analysis.

---

## Hands-on Lab

### Lab 8.1: Forward Kinematics Implementation

**TODO**: Python code to compute end-effector pose from joint angles using numpy transformation matrices.

### Lab 8.2: Inverse Kinematics with MoveIt2

**TODO**: Use MoveIt2's KDL IK solver to reach target (x, y, z), visualize in RViz, handle unreachable poses.

### Lab 8.3: Jacobian-Based Velocity Control

**TODO**: Implement Jacobian pseudoinverse for real-time Cartesian velocity tracking (dx/dt → dθ/dt).

---

## Assessment

<MCQ
  id="ch08-mcq-01"
  question="What is the purpose of Denavit-Hartenberg (DH) parameters in robot kinematics?"
  options={[
    "To compress URDF files for faster loading",
    "To systematically define coordinate frames and compute forward kinematics via 4x4 transformation matrices",
    "To control joint velocities in real-time",
    "To detect collisions between robot links"
  ]}
  correctIndex={1}
  explanation="Correct! DH parameters (a, α, d, θ) define each joint's coordinate frame transformation. Multiplying DH matrices gives the end-effector pose (forward kinematics). See Section 8.1."
  difficulty="medium"
/>

<MCQ
  id="ch08-mcq-02"
  question="Why is inverse kinematics (IK) harder than forward kinematics (FK) for humanoid robots?"
  options={[
    "FK is harder because it requires solving nonlinear equations",
    "IK may have multiple solutions, no solution (unreachable pose), or singularities, whereas FK is a direct computation",
    "IK always runs slower than FK",
    "IK requires expensive GPUs, FK runs on CPUs"
  ]}
  correctIndex={1}
  explanation="Correct! FK is straightforward (plug joint angles into DH matrices → get pose). IK solves: given target pose, find joint angles—this can have 0 solutions (out of workspace), infinite solutions (redundant arm), or numerical instability near singularities. See Section 8.2."
  difficulty="medium"
/>

<MCQ
  id="ch08-mcq-03"
  question="What does a Jacobian matrix represent in robotics?"
  options={[
    "The relationship between joint velocities (dθ/dt) and end-effector velocities (dx/dt, dy/dt, dz/dt)",
    "The mass and inertia properties of robot links",
    "The path planning algorithm used by MoveIt2",
    "The maximum torque each joint can exert"
  ]}
  correctIndex={0}
  explanation="Correct! The Jacobian J maps joint space velocities to Cartesian space velocities: ẋ = J(θ) * θ̇. It's essential for velocity control and analyzing singularities (when J loses rank). See Section 8.3."
  difficulty="medium"
/>

<MCQ
  id="ch08-mcq-04"
  question="What happens when a robot arm reaches a singularity?"
  options={[
    "The arm instantly stops moving",
    "The Jacobian matrix becomes singular (determinant = 0), causing loss of controllability in certain directions",
    "The arm's joints exceed their position limits",
    "MoveIt2 automatically generates a safe recovery path"
  ]}
  correctIndex={1}
  explanation="Correct! At singularities (e.g., elbow fully extended), the Jacobian loses rank, meaning the robot cannot move in certain Cartesian directions no matter how fast the joints move. This causes instability in IK solvers. See Section 8.3 singularity analysis."
  difficulty="hard"
/>

<MCQ
  id="ch08-mcq-05"
  question="How does MoveIt2's KDL IK solver handle redundant arms (>6 DOF)?"
  options={[
    "It randomly picks one of the infinite IK solutions",
    "It uses numerical optimization to find solutions that minimize joint displacement or stay within preferred ranges",
    "It only works with 6-DOF arms (non-redundant)",
    "It requires manual specification of the 7th joint angle"
  ]}
  correctIndex={1}
  explanation="Correct! For redundant arms (7+ DOF for 6-DOF tasks), KDL uses iterative optimization (e.g., minimize joint changes, avoid joint limits, maintain human-like postures). You can configure cost functions in MoveIt2 config YAML. See Lab 8.2 MoveIt2 IK."
  difficulty="hard"
/>

---

## Summary

In this chapter, you learned:

1. **Forward kinematics**: DH parameters, transformation matrices, end-effector pose
2. **Inverse kinematics**: Analytical (closed-form) vs. numerical (Jacobian-based), redundancy resolution
3. **Jacobian**: Velocity mapping, singularity detection, damped least squares for stability
4. **MoveIt2**: Motion planning with collision avoidance, IK plugin configuration
5. **Optimization**: Human-like posture costs, joint limit avoidance

**Key Takeaways**:
- FK is direct, IK is inverse and often multiple solutions exist
- Jacobian reveals velocity relationships and singularities
- MoveIt2 handles complex IK with obstacles and constraints

**Next Steps**: In [Chapter 9: Locomotion](/docs/ch09-locomotion), you'll apply kinematics to bipedal walking with ZMP control.

---

## Further Reading

- [MoveIt2 IK Documentation](https://moveit.picknik.ai/main/doc/examples/kinematics/kinematics.html)
- Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2006). *Robot Modeling and Control*. Wiley.

---

**Chapter 8 Complete** | Next: [Chapter 9: Bipedal Locomotion and ZMP Control](/docs/ch09-locomotion)

---

**TODO**: Expand theory (800 words each), complete labs, add 3-5 MCQs
