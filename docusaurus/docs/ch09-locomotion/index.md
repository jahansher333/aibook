---
id: ch09
title: "Bipedal Locomotion: ZMP Control and Trajectory Optimization"
sidebar_position: 9
week: 9
objectives:
  - "Understand Zero Moment Point (ZMP) stability criterion for bipedal walking"
  - "Design gait patterns with footstep planning and swing/stance phases"
  - "Implement ZMP-based balance control for dynamic walking"
  - "Use trajectory optimization (CHOMP, TrajOpt) for collision-free locomotion"
  - "Simulate bipedal walking in Isaac Sim with disturbance rejection"
tags: [locomotion, zmp, bipedal, gait, trajectory-optimization, balance]
description: "Master bipedal locomotion: ZMP stability, gait generation, trajectory optimization, and balance control for humanoid walking."
---

import MCQ from '@site/src/components/MCQ';
import PersonalizeButton from '@site/src/components/PersonalizeButton/PersonalizeButton';
import UrduTranslationButton from '@site/src/components/UrduTranslationButton';

<PersonalizeButton chapterId="ch09" chapterContent="Bipedal Locomotion" />

<UrduTranslationButton chapterId="ch09" />

# Bipedal Locomotion: ZMP Control and Trajectory Optimization

## Learning Objectives

1. **Understand** Zero Moment Point (ZMP) stability criterion for bipedal walking
2. **Design** gait patterns with footstep planning and swing/stance phases
3. **Implement** ZMP-based balance control for dynamic walking
4. **Use** trajectory optimization (CHOMP, TrajOpt) for collision-free locomotion
5. **Simulate** bipedal walking in Isaac Sim with disturbance rejection

---

## Theory

### 9.1 Zero Moment Point (ZMP) and Stability

**TODO**: Explain ZMP definition (point where net moment is zero), support polygon, ZMP within polygon = stable, outside = falling.

### 9.2 Gait Generation and Footstep Planning

**TODO**: Walking cycle (double support, single support, swing phase), foot trajectory planning (polynomial, spline), step length/width/height.

### 9.3 Trajectory Optimization for Humanoids

**TODO**: CHOMP (Covariant Hamiltonian Optimization for Motion Planning), collision cost, smoothness regularization, gradient descent.

---

## Hands-on Lab

### Lab 9.1: ZMP Calculation for Static Walking

**TODO**: Python code to compute ZMP from joint torques and contact forces, visualize ZMP trajectory relative to support polygon.

### Lab 9.2: Gait Pattern Design in Isaac Sim

**TODO**: Implement walking gait for 5-link bipedal robot (hip, knee, ankle), simulate in Isaac Sim, tune step frequency (0.5 Hz → 1 Hz).

### Lab 9.3: Balance Control with Push Recovery

**TODO**: Apply lateral push (50N impulse) during walking, use ZMP feedback to adjust ankle torques, demonstrate recovery without falling.

---

## Assessment

<MCQ
  id="ch09-mcq-01"
  question="What does it mean for the Zero Moment Point (ZMP) to be inside the support polygon?"
  options={[
    "The robot is statically stable (no risk of tipping over)",
    "The robot is moving at maximum speed",
    "The robot's center of mass is exactly at the ZMP location",
    "The robot has at least 3 legs on the ground"
  ]}
  correctIndex={0}
  explanation="Correct! When ZMP is inside the support polygon (convex hull of contact points), the net moment about the ZMP is zero, meaning the robot won't tip. If ZMP exits the polygon, unbalanced moments cause falling. See Section 9.1 ZMP stability criterion."
  difficulty="medium"
/>

<MCQ
  id="ch09-mcq-02"
  question="During bipedal walking, what is the 'swing phase'?"
  options={[
    "The period when both feet are on the ground (double support)",
    "The period when one foot is in the air, moving forward to the next step (single support on other leg)",
    "The initial acceleration phase when the robot starts walking from rest",
    "The phase when the robot's arms swing to balance"
  ]}
  correctIndex={1}
  explanation="Correct! Swing phase is when one leg leaves the ground and swings forward, while the stance leg supports the entire body weight (single support). Double support (both feet down) occurs briefly during foot transitions. See Section 9.2 gait cycle phases."
  difficulty="easy"
/>

<MCQ
  id="ch09-mcq-03"
  question="Why is trajectory optimization (CHOMP, TrajOpt) necessary for humanoid locomotion in cluttered environments?"
  options={[
    "To generate smooth, collision-free trajectories that avoid obstacles while maintaining balance and joint limits",
    "To reduce computational cost by simplifying the robot model",
    "To eliminate the need for sensors like cameras and LiDAR",
    "To automatically tune PID controller gains"
  ]}
  correctIndex={0}
  explanation="Correct! CHOMP optimizes trajectories to minimize collision cost (distance to obstacles) and smoothness cost (jerk, acceleration), subject to kinematic constraints (joint limits, ZMP stability). This enables walking around furniture or over obstacles. See Section 9.3 trajectory optimization."
  difficulty="medium"
/>

<MCQ
  id="ch09-mcq-04"
  question="What is the typical walking frequency (step rate) for humanoid robots?"
  options={[
    "10 Hz (10 steps per second, very fast running)",
    "0.5-1 Hz (one step every 1-2 seconds, slow to moderate walking)",
    "0.01 Hz (one step every 100 seconds, extremely slow)",
    "5 Hz (5 steps per second, human sprint speed)"
  ]}
  correctIndex={1}
  explanation="Correct! Humanoids typically walk at 0.5-1 Hz (step period: 1-2 seconds), similar to slow human walking. Faster gaits (>1.5 Hz) require dynamic balance (ZMP may temporarily exit support polygon, relying on momentum). See Lab 9.2 step frequency tuning."
  difficulty="easy"
/>

<MCQ
  id="ch09-mcq-05"
  question="How does a humanoid robot recover from a lateral push during walking?"
  options={[
    "It immediately stops walking and goes into a crouch",
    "It adjusts ankle torques based on ZMP error feedback to shift the ZMP back inside the support polygon",
    "It increases walking speed to outrun the disturbance",
    "It uses its arms to push against nearby objects for support"
  ]}
  correctIndex={1}
  explanation="Correct! When pushed, the ZMP shifts toward the polygon edge (risk of tipping). Ankle torque control (PD controller on ZMP error) shifts the ZMP back toward the center. If disturbance is too large, the robot may take a step sideways (capture point strategy). See Lab 9.3 push recovery demo."
  difficulty="hard"
/>

---

## Summary

In this chapter, you learned:

1. **ZMP stability**: ZMP inside support polygon = stable, outside = falling
2. **Gait generation**: Footstep planning, swing/stance phases, foot trajectory design
3. **Trajectory optimization**: CHOMP for collision-free, smooth locomotion
4. **Balance control**: ZMP feedback, ankle torque adjustment, disturbance rejection
5. **Simulation**: Isaac Sim bipedal walker with push recovery

**Key Takeaways**:
- ZMP is the foundation of bipedal stability theory
- Walking is a sequence of controlled falls (dynamic balance)
- Trajectory optimization enables navigation in complex environments

**Next Steps**: In [Chapter 10: Manipulation](/docs/ch10-manipulation), you'll apply kinematics to grasping and object manipulation with force control.

---

## Further Reading

- Kajita, S., et al. (2003). *Biped walking pattern generation by using preview control of zero-moment point*. IEEE ICRA.
- [CHOMP Paper](http://www.nathanratliff.com/research/papers/CHOMP.pdf)

---

**Chapter 9 Complete** | Next: [Chapter 10: Manipulation with Grasping and Force Control](/docs/ch10-manipulation)

---

**TODO**: Expand theory, complete labs, add 3-5 MCQs
