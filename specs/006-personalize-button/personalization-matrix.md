# Personalization Matrix: Personalize Button per Chapter

**Feature**: Personalize Button per Chapter
**Created**: 2025-12-10
**Matrix Version**: 1.0

## Overview

This matrix defines how user profile attributes map to content personalization rules across all 13 chapters. Each profile dimension (learning environment, ROS2 knowledge, etc.) triggers specific content transformations in chapters.

## Profile Dimensions & Values

### 1. Learning Environment (`learning_environment`)
- `cloud_only`: User has no local hardware access
- `cloud_preferred`: User prefers cloud but has local options
- `local_preferred`: User prefers local setup but cloud is available
- `local_only`: User has local hardware only

### 2. ROS2 Knowledge (`ros2_knowledge`)
- `none`: No prior ROS experience
- `basic`: Familiar with ROS/ROS2 basics
- `intermediate`: Comfortable with ROS2 nodes, topics, services
- `advanced`: Expert in ROS2 architecture, performance

### 3. Learning Goal (`learning_goal`)
- `academic`: Learning for educational purposes
- `hobby`: Learning for personal projects
- `career_transition`: Learning to enter robotics field
- `professional`: Learning for work projects

### 4. Hardware Experience (`hardware_experience`)
- `none`: No hardware experience
- `some`: Some basic hardware experience
- `proficient`: Comfortable with hardware assembly
- `expert`: Expert in hardware troubleshooting

### 5. GPU Access (`gpu_access`)
- `none`: No GPU access
- `consumer`: Consumer-grade GPU (GTX/RTX 20xx, 30xx, 40xx)
- `midrange`: Mid-range GPU (RTX 4090, A5000, etc.)
- `highend`: High-end GPU (multiple A100, H100, etc.)

### 6. Python Level (`python_level`)
- `beginner`: Basic Python knowledge
- `intermediate`: Comfortable with Python
- `advanced`: Advanced Python (async, decorators, etc.)
- `expert`: Python expert (metaclasses, etc.)

### 7. Learning Environment (`learning_environment`)
- `cloud_only`: User has no local hardware access
- `cloud_preferred`: User prefers cloud but has local options
- `local_preferred`: User prefers local setup but cloud is available
- `local_only`: User has local hardware only

## Personalization Rules Matrix

### Chapter 1: Introduction to AI Robotics
| Profile Dimension | Value | Content Transformation |
|-------------------|-------|------------------------|
| ROS2 Knowledge | `none` | Add ROS2 primer section at start with definitions and links |
| ROS2 Knowledge | `basic` | Add brief refresher with advanced concepts links |
| ROS2 Knowledge | `intermediate/advanced` | Assume knowledge, link to advanced patterns |
| Learning Goal | `academic` | Highlight learning objectives and assessment sections |
| Learning Goal | `hobby` | Emphasize fun projects and weekend-friendly timelines |
| Learning Goal | `career_transition` | Link to job descriptions and industry best practices |
| Learning Goal | `professional` | Focus on workplace context and team collaboration |

### Chapter 2: Setting Up Development Environment
| Profile Dimension | Value | Content Transformation |
|-------------------|-------|------------------------|
| Learning Environment | `cloud_only/cloud_preferred` | Prioritize cloud setup (AWS EC2), emphasize cost estimates, hide local installation |
| Learning Environment | `local_preferred/local_only` | Prioritize local installation, show cloud as alternative, emphasize hardware requirements |
| Hardware Experience | `none/some` | Simplify hardware requirements, add beginner callouts |
| Hardware Experience | `proficient/expert` | Show advanced hardware options, assume knowledge |
| GPU Access | `none/consumer` | Prioritize cloud instructions, warn about local performance |
| GPU Access | `midrange` | Show both local and cloud options equally |
| GPU Access | `highend` | Default to local setup, emphasize GPU features |

### Chapter 3: Understanding ROS2 Architecture
| Profile Dimension | Value | Content Transformation |
|-------------------|-------|------------------------|
| ROS2 Knowledge | `none` | Add comprehensive ROS2 primer section at start |
| ROS2 Knowledge | `basic` | Show brief refresher with specific documentation links |
| ROS2 Knowledge | `intermediate/advanced` | Assume knowledge, link to advanced patterns |
| Python Level | `beginner` | Add Python syntax explanations in code comments |
| Python Level | `intermediate` | Standard code examples with brief comments |
| Python Level | `advanced/expert` | Show Pythonic alternatives, assume modern Python knowledge |
| Learning Goal | `academic` | Include theoretical concepts and academic references |
| Learning Goal | `hobby` | Focus on practical examples and project applications |

### Chapter 4: Isaac Sim Fundamentals
| Profile Dimension | Value | Content Transformation |
|-------------------|-------|------------------------|
| Learning Environment | `cloud_only/cloud_preferred` | Prioritize cloud-based Isaac Sim access, emphasize AWS G4DN instances |
| Learning Environment | `local_preferred/local_only` | Prioritize local Isaac Sim installation, system requirements |
| GPU Access | `none/consumer` | Emphasize cloud-based options, warn about performance |
| GPU Access | `midrange` | Show local/cloud balance, performance optimization tips |
| GPU Access | `highend` | Focus on advanced GPU features and optimization |
| Hardware Experience | `none/some` | Simplify hardware setup instructions |
| Hardware Experience | `proficient/expert` | Show advanced configuration options |

### Chapter 5: Navigation and Path Planning
| Profile Dimension | Value | Content Transformation |
|-------------------|-------|------------------------|
| ROS2 Knowledge | `none/basic` | Add ROS2 navigation basics primer |
| ROS2 Knowledge | `intermediate/advanced` | Focus on advanced navigation concepts |
| Learning Goal | `academic` | Include theoretical background and research papers |
| Learning Goal | `hobby` | Emphasize practical examples and simple implementations |
| Learning Goal | `career_transition` | Include industry best practices and deployment considerations |
| Python Level | `beginner` | Add detailed code explanations |
| Python Level | `advanced/expert` | Show optimized implementations |

### Chapter 6: Perception and Computer Vision
| Profile Dimension | Value | Content Transformation |
|-------------------|-------|------------------------|
| Learning Environment | `cloud_only` | Emphasize cloud-based CV services, remote processing |
| Learning Environment | `local_only` | Focus on local CV implementation, hardware requirements |
| GPU Access | `none` | Emphasize CPU-based implementations, cloud alternatives |
| GPU Access | `consumer/midrange` | Show balanced GPU/CPU implementations |
| GPU Access | `highend` | Focus on GPU-optimized CV pipelines |
| Python Level | `beginner` | Add CV library usage explanations |
| Python Level | `advanced/expert` | Show advanced CV techniques |

### Chapter 7: Manipulation and Control
| Profile Dimension | Value | Content Transformation |
|-------------------|-------|------------------------|
| Hardware Experience | `none/some` | Simplify hardware control explanations |
| Hardware Experience | `proficient/expert` | Show advanced control techniques |
| Learning Goal | `academic` | Include control theory and mathematical models |
| Learning Goal | `hobby` | Focus on practical manipulation tasks |
| Learning Goal | `professional` | Emphasize industrial control patterns |
| ROS2 Knowledge | `none/basic` | Add control node basics primer |
| ROS2 Knowledge | `advanced` | Focus on advanced control patterns |

### Chapter 8: SLAM (Simultaneous Localization and Mapping)
| Profile Dimension | Value | Content Transformation |
|-------------------|-------|------------------------|
| ROS2 Knowledge | `none/basic` | Add SLAM basics primer with ROS2 examples |
| ROS2 Knowledge | `intermediate/advanced` | Focus on advanced SLAM techniques |
| Learning Environment | `cloud_only` | Emphasize simulation-based SLAM learning |
| Learning Environment | `local_preferred` | Focus on hardware-based SLAM |
| GPU Access | `none` | Emphasize lightweight SLAM algorithms |
| GPU Access | `highend` | Show GPU-accelerated SLAM implementations |
| Learning Goal | `academic` | Include mathematical foundations |

### Chapter 9: AI Integration in Robotics
| Profile Dimension | Value | Content Transformation |
|-------------------|-------|------------------------|
| Python Level | `beginner` | Add AI library usage tutorials |
| Python Level | `advanced/expert` | Show advanced AI integration patterns |
| Learning Goal | `hobby` | Focus on fun AI applications (object recognition, etc.) |
| Learning Goal | `career_transition` | Include industry AI deployment practices |
| Learning Goal | `professional` | Emphasize production AI systems |
| GPU Access | `none` | Focus on lightweight AI models |
| GPU Access | `highend` | Show large model deployment |

### Chapter 10: Simulation to Real World
| Profile Dimension | Value | Content Transformation |
|-------------------|-------|------------------------|
| Hardware Experience | `none/some` | Simplify hardware transition explanations |
| Hardware Experience | `proficient/expert` | Show complex hardware integration |
| Learning Environment | `cloud_only` | Focus on remote hardware access |
| Learning Environment | `local_only` | Emphasize direct hardware interaction |
| Learning Goal | `academic` | Include simulation accuracy research |
| Learning Goal | `hobby` | Focus on personal hardware projects |
| Learning Goal | `professional` | Emphasize industrial deployment |

### Chapter 11: Multi-Robot Systems
| Profile Dimension | Value | Content Transformation |
|-------------------|-------|------------------------|
| ROS2 Knowledge | `none/basic` | Add multi-robot ROS2 basics primer |
| ROS2 Knowledge | `advanced` | Focus on advanced coordination patterns |
| Learning Goal | `academic` | Include distributed systems theory |
| Learning Goal | `professional` | Emphasize industrial multi-robot deployment |
| Learning Goal | `hobby` | Focus on simple multi-robot projects |
| Python Level | `beginner` | Add network programming basics |
| Python Level | `advanced` | Show optimized communication patterns |

### Chapter 12: Hardware Deployment
| Profile Dimension | Value | Content Transformation |
|-------------------|-------|------------------------|
| Hardware Experience | `none/some` | Simplify hardware setup and deployment |
| Hardware Experience | `proficient/expert` | Show advanced deployment options |
| Learning Goal | `career_transition` | Focus on professional deployment practices |
| Learning Goal | `professional` | Emphasize production deployment |
| Learning Goal | `hobby` | Focus on personal project deployment |
| GPU Access | `none` | Emphasize lightweight deployment options |
| GPU Access | `highend` | Show high-performance deployment |
| Learning Environment | `cloud_only` | Focus on cloud deployment |
| Learning Environment | `local_only` | Emphasize on-premises deployment |

### Chapter 13: Performance Optimization and Debugging
| Profile Dimension | Value | Content Transformation |
|-------------------|-------|------------------------|
| Python Level | `beginner` | Add debugging tools and techniques tutorials |
| Python Level | `advanced/expert` | Show advanced profiling and optimization |
| Learning Goal | `academic` | Include performance analysis theory |
| Learning Goal | `professional` | Focus on production debugging |
| ROS2 Knowledge | `none/basic` | Add ROS2 debugging tools primer |
| ROS2 Knowledge | `advanced` | Focus on advanced ROS2 debugging |
| GPU Access | `none` | Focus on CPU optimization techniques |
| GPU Access | `highend` | Show GPU profiling and optimization |

## Default Profile for Anonymous Users

When no profile is available (anonymous users or incomplete profiles), the system uses default values:

| Dimension | Default Value | Reasoning |
|-----------|---------------|-----------|
| `learning_environment` | `cloud_preferred` | Cloud is more accessible to beginners |
| `ros2_knowledge` | `basic` | Assumes some basic knowledge |
| `learning_goal` | `hobby` | Hobby learners are most common |
| `hardware_experience` | `some` | Assumes basic computer hardware knowledge |
| `gpu_access` | `consumer` | Mid-range consumer GPUs are common |
| `python_level` | `intermediate` | Assumes basic programming knowledge |
| `learning_goal` | `hobby` | Hobby projects are common entry point |

## Content Transformation Types

Each personalization rule applies one or more of these transformations:

1. **Section Visibility**: Show/hide specific content sections
2. **Content Emphasis**: Highlight important sections for user profile
3. **Text Replacement**: Replace technical terms with user-appropriate alternatives
4. **Resource Links**: Provide profile-appropriate links and references
5. **Code Examples**: Show code examples matching user's skill level
6. **Hardware Recommendations**: Suggest appropriate hardware based on profile

## Validation Criteria

Each personalization rule must satisfy:
- **Relevance**: Content is relevant to user's profile
- **Accessibility**: Content matches user's skill level
- **Practicality**: Content fits user's environment and resources
- **Consistency**: Similar profiles receive similar content transformations
- **Transparency**: User understands why content was personalized

## Matrix Evolution

This matrix should be updated when:
- New profile dimensions are added
- User feedback indicates ineffective personalization
- New chapters are added to the curriculum
- Content structure changes significantly