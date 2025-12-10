# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/1-physical-ai-textbook/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Organization**: Tasks are grouped by phase to enable systematic implementation and testing.

## Format: `[ID] [Priority] [Parallelizable] Description`

- **[P1]**: Critical path, must complete first
- **[P2]**: Core features, after foundational setup
- **[P3]**: Enhancements, after core complete
- **[P]**: Can run in parallel (different files, no dependencies)
- **Time estimates**: Approximate duration for planning

---

## Phase 1: Project Setup & Infrastructure [P1] 🎯 MVP Foundation

**Purpose**: Initialize Docusaurus project, configure dependencies, setup CI/CD

**Estimated Time**: 2-3 hours

- [ ] **T001** [P1] Create Docusaurus project with Classic preset and TypeScript
  ```bash
  npx create-docusaurus@latest docusaurus classic --typescript
  cd docusaurus
  npm install
  ```
  **Time**: 15 min
  **Acceptance**: `docusaurus/` folder created with package.json, docs/, src/, static/

- [ ] **T002** [P1] Install additional dependencies (Mermaid theme, React testing)
  ```bash
  cd docusaurus
  npm install @docusaurus/theme-mermaid@latest
  npm install --save-dev @testing-library/react @testing-library/jest-dom jest ts-jest @types/jest
  ```
  **Time**: 5 min
  **Acceptance**: Dependencies added to package.json, node_modules/ updated

- [ ] **T003** [P1] Configure docusaurus.config.js (site metadata, Mermaid, i18n stub)
  ```javascript
  // Edit docusaurus/docusaurus.config.js
  module.exports = {
    title: 'Physical AI & Humanoid Robotics Textbook',
    tagline: 'Learn ROS 2, Isaac Sim, and VLA Models for Humanoid Robots',
    url: 'https://<username>.github.io',
    baseUrl: '/physical-ai-textbook/',
    organizationName: '<username>',
    projectName: 'physical-ai-textbook',
    onBrokenLinks: 'throw',
    onBrokenMarkdownLinks: 'warn',
    favicon: 'img/favicon.ico',

    themes: ['@docusaurus/theme-mermaid'],
    markdown: {
      mermaid: true,
    },

    i18n: {
      defaultLocale: 'en',
      locales: ['en', 'ur'],
      localeConfigs: {
        en: { label: 'English', direction: 'ltr' },
        ur: { label: 'اردو', direction: 'rtl' },
      },
    },

    themeConfig: {
      navbar: {
        title: 'Physical AI Textbook',
        logo: { alt: 'Logo', src: 'img/logo.svg' },
        items: [
          { type: 'doc', docId: 'intro', position: 'left', label: 'Chapters' },
          { to: '/hardware-requirements', label: 'Hardware', position: 'left' },
          { type: 'localeDropdown', position: 'right' },
          { href: 'https://github.com/<org>/physical-ai-textbook', label: 'GitHub', position: 'right' },
        ],
      },
      footer: {
        style: 'dark',
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer/themes/github'),
        darkTheme: require('prism-react-renderer/themes/dracula'),
        additionalLanguages: ['python', 'bash', 'yaml', 'xml'],
      },
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
      mermaid: {
        theme: { light: 'default', dark: 'dark' },
      },
    },
  };
  ```
  **Time**: 20 min
  **Acceptance**: Site metadata correct, Mermaid enabled, i18n configured with ur locale

- [ ] **T004** [P1] Configure sidebars.js with 13 chapters + Hardware page
  ```javascript
  // Edit docusaurus/sidebars.js
  module.exports = {
    docs: [
      {
        type: 'doc',
        id: 'intro',
        label: 'Welcome',
      },
      {
        type: 'category',
        label: 'Chapters',
        collapsed: false,
        items: [
          'ch01-physical-ai-intro/index',
          'ch02-ros2-fundamentals/index',
          'ch03-robot-modeling/index',
          'ch04-gazebo-simulation/index',
          'ch05-unity-simulation/index',
          'ch06-isaac-sim/index',
          'ch07-vla-models/index',
          'ch08-humanoid-kinematics/index',
          'ch09-locomotion/index',
          'ch10-manipulation/index',
          'ch11-conversational-ai/index',
          'ch12-hardware-integration/index',
          'ch13-capstone-project/index',
        ],
      },
      {
        type: 'doc',
        id: 'hardware-requirements',
        label: 'Hardware Requirements',
      },
    ],
  };
  ```
  **Time**: 10 min
  **Acceptance**: Sidebar configured with 13 chapters, hardware page, auto-collapsed

- [ ] **T005** [P1] Create i18n stub directory for Urdu translations
  ```bash
  cd docusaurus
  mkdir -p i18n/ur/docusaurus-plugin-content-docs/current
  echo '{}' > i18n/ur/code.json
  ```
  **Time**: 5 min
  **Acceptance**: i18n/ur/ folder exists with empty code.json

- [ ] **T006** [P1] Create GitHub Actions workflow for deployment
  ```bash
  mkdir -p ../.github/workflows
  cat > ../.github/workflows/deploy.yml << 'EOF'
  name: Deploy Physical AI Textbook to GitHub Pages

  on:
    push:
      branches: [main]
    pull_request:
      branches: [main]
    workflow_dispatch:

  permissions:
    contents: read
    pages: write
    id-token: write

  concurrency:
    group: "pages"
    cancel-in-progress: true

  jobs:
    build:
      name: Build Docusaurus Site
      runs-on: ubuntu-latest

      steps:
        - name: Checkout repository
          uses: actions/checkout@v4
          with:
            fetch-depth: 0

        - name: Setup Node.js 20
          uses: actions/setup-node@v4
          with:
            node-version: '20'
            cache: 'npm'
            cache-dependency-path: docusaurus/package-lock.json

        - name: Install dependencies
          run: |
            cd docusaurus
            npm ci

        - name: Build Docusaurus site
          run: |
            cd docusaurus
            npm run build
          env:
            NODE_ENV: production

        - name: Run Lighthouse CI (Performance & Accessibility)
          run: |
            cd docusaurus
            npm install -g @lhci/cli
            lhci autorun --collect.staticDistDir=./build || echo "Lighthouse CI warning (non-blocking)"
          continue-on-error: true

        - name: Upload build artifact
          uses: actions/upload-pages-artifact@v3
          with:
            path: ./docusaurus/build

    deploy:
      name: Deploy to GitHub Pages
      needs: build
      runs-on: ubuntu-latest
      if: github.ref == 'refs/heads/main' && github.event_name == 'push'

      environment:
        name: github-pages
        url: ${{ steps.deployment.outputs.page_url }}

      steps:
        - name: Deploy to GitHub Pages
          id: deployment
          uses: actions/deploy-pages@v4
  EOF
  ```
  **Time**: 15 min
  **Acceptance**: .github/workflows/deploy.yml exists, triggers on push to main, deploys to Pages

- [ ] **T007** [P1] Test local development server
  ```bash
  cd docusaurus
  npm run start
  # Open http://localhost:3000/ - verify site loads
  # Press Ctrl+C to stop
  ```
  **Time**: 5 min
  **Acceptance**: Site loads at localhost:3000, no build errors

**Checkpoint**: Foundation ready - Docusaurus configured, CI/CD in place, ready for content creation

---

## Phase 2: Chapter Content Structure [P1] 🎯 MVP Content

**Purpose**: Create 13 chapter folders with MDX templates (Objectives, Theory, Lab, Assessment sections)

**Estimated Time**: 3-4 hours

- [ ] **T008** [P1] [P] Create intro.md landing page
  ```bash
  cd docusaurus
  cat > docs/intro.md << 'EOF'
  ---
  id: intro
  title: Welcome to Physical AI & Humanoid Robotics
  sidebar_position: 1
  slug: /
  ---

  # Welcome to Physical AI & Humanoid Robotics

  Learn to build intelligent humanoid robots using ROS 2, Isaac Sim, and Vision-Language-Action (VLA) models.

  ## What You'll Learn

  - **ROS 2 Fundamentals**: Nodes, topics, services, and launch files
  - **Simulation Environments**: Gazebo, Unity, NVIDIA Isaac Sim
  - **Humanoid Robotics**: Kinematics, locomotion, manipulation
  - **AI Integration**: VLA models with Whisper, GPT-4, and CLIP
  - **Hardware Deployment**: Jetson Orin Nano, RealSense sensors

  ## Course Structure

  - **13 Chapters**: Week-by-week progression from basics to capstone
  - **Hands-on Labs**: Executable code for simulation and hardware
  - **Assessments**: MCQ quizzes to test understanding
  - **Hardware Guides**: Three budget tiers (Economy $700, Proxy $3k, Cloud $205/quarter)

  ## Get Started

  Navigate to **Chapter 1** in the sidebar to begin your journey into Physical AI!
  EOF
  ```
  **Time**: 10 min
  **Acceptance**: docs/intro.md exists, renders as landing page at /

- [ ] **T009** [P1] [P] Create Chapter 1: Introduction to Physical AI
  ```bash
  cd docusaurus
  mkdir -p docs/ch01-physical-ai-intro
  cat > docs/ch01-physical-ai-intro/index.md << 'EOF'
  ---
  id: ch01
  title: "Introduction to Physical AI and Embodied Intelligence"
  sidebar_position: 2
  week: 1
  objectives:
    - "Understand the concept of Physical AI and embodied intelligence"
    - "Differentiate between digital AI and physical robotics systems"
    - "Identify key components of humanoid robot architectures"
  tags: [physical-ai, embodied-intelligence, humanoid-robotics, introduction]
  description: "Introduction to Physical AI: Learn how intelligent systems interact with the physical world through embodied intelligence and humanoid robotics."
  ---

  # Introduction to Physical AI and Embodied Intelligence

  ## Objectives

  By the end of this chapter, you will be able to:

  - Understand the concept of Physical AI and embodied intelligence
  - Differentiate between digital AI and physical robotics systems
  - Identify key components of humanoid robot architectures

  ---

  ## Theory

  ### What is Physical AI?

  **Physical AI** refers to artificial intelligence systems that interact with and manipulate the physical world through robotic embodiment. Unlike traditional AI (digital assistants, recommendation systems), Physical AI requires:

  1. **Sensors**: Cameras, LiDAR, force sensors to perceive environment
  2. **Actuators**: Motors, grippers to manipulate objects
  3. **Real-time Control**: Low-latency decision-making (<100ms for safety)
  4. **Sim-to-Real Transfer**: Training in simulation, deploying to hardware

  ```mermaid
  graph LR
      A[Sensors - Perception] --> B[AI Brain - Planning]
      B --> C[Actuators - Action]
      C --> D[Physical World]
      D --> A
  ```

  ### Embodied Intelligence

  **Embodied intelligence** emphasizes that intelligence emerges from interaction between body, brain, and environment ([Brooks, 1991](https://people.csail.mit.edu/brooks/papers/AIM-1293.pdf)).

  Key principles:
  - **Situatedness**: Agent acts in real-world context (not abstract symbols)
  - **Morphology**: Body shape influences capabilities (bipedal vs. quadruped)
  - **Sensorimotor loops**: Tight coupling between sensing and acting

  ### Humanoid Robots

  Humanoid robots mimic human form (head, torso, two arms, two legs) to:
  - Navigate human-designed spaces (stairs, doorways)
  - Use human tools (hammers, keyboards)
  - Communicate naturally (gestures, facial expressions)

  **Examples**:
  - **Boston Dynamics Atlas**: 28 DOF, backflip-capable
  - **Unitree G1**: $16k, 23 DOF, 1.3m tall
  - **Tesla Optimus**: Under development, $20k target

  ---

  ## Hands-on Lab

  ### Prerequisites

  - **Software**: None (conceptual chapter)
  - **Hardware**: None (theory only)

  ### Activity: Analyze a Humanoid Robot Video

  1. Watch Boston Dynamics Atlas demo: [https://www.youtube.com/watch?v=tF4DML7FIWk](https://www.youtube.com/watch?v=tF4DML7FIWk)
  2. Identify:
     - Sensors used (cameras, IMU, force sensors)
     - Actuators (hydraulic joints)
     - Control challenges (balance, obstacle avoidance)
  3. Write 3-sentence analysis: "Atlas uses [sensors] to perceive [environment], [actuators] to [action], and overcomes [challenge] by [method]."

  **Expected Output**: Written analysis demonstrating understanding of Physical AI components.

  ---

  ## Assessment

  import MCQ from '@site/src/components/MCQ';

  <MCQ
    id="ch01-mcq-01"
    question="What distinguishes Physical AI from traditional digital AI?"
    options={[
      "Physical AI requires interaction with the physical world through sensors and actuators",
      "Physical AI runs on faster processors",
      "Physical AI uses more data for training",
      "Physical AI only works in simulation"
    ]}
    correctIndex={0}
    explanation="Correct! Physical AI distinguishes itself by requiring embodied interaction with the physical world via sensors (perception) and actuators (manipulation), unlike digital AI which operates on abstract data."
    difficulty="easy"
  />

  <MCQ
    id="ch01-mcq-02"
    question="Which principle is NOT part of embodied intelligence?"
    options={[
      "Situatedness (acting in real-world context)",
      "Morphology (body shape influences capabilities)",
      "Cloud processing (offloading computation to servers)",
      "Sensorimotor loops (tight coupling between sensing and acting)"
    ]}
    correctIndex={2}
    explanation="Correct! Cloud processing is not a principle of embodied intelligence. Embodied intelligence emphasizes real-time, local sensorimotor loops. Cloud processing introduces latency unsuitable for real-time control (see constitution safety warnings)."
    difficulty="medium"
  />

  <MCQ
    id="ch01-mcq-03"
    question="Why are humanoid robots designed to mimic human form?"
    options={[
      "To look friendly and non-threatening",
      "To navigate human-designed spaces and use human tools",
      "To reduce manufacturing costs",
      "To simplify control algorithms"
    ]}
    correctIndex={1}
    explanation="Correct! Humanoid form allows robots to navigate environments designed for humans (stairs, doorways) and use human tools without redesigning infrastructure."
    difficulty="easy"
  />

  <MCQ
    id="ch01-mcq-04"
    question="What is sim-to-real transfer in Physical AI?"
    options={[
      "Moving files from simulation to real robot",
      "Training AI in simulation, then deploying to physical hardware",
      "Simulating real-world physics in software",
      "Controlling real robots from cloud simulation"
    ]}
    correctIndex={1}
    explanation="Correct! Sim-to-real transfer involves training AI policies in safe, scalable simulation environments (Isaac Sim, Gazebo), then deploying trained models to physical robots."
    difficulty="medium"
  />

  <MCQ
    id="ch01-mcq-05"
    question="What is the typical control latency requirement for safe Physical AI?"
    options={[
      "<10ms",
      "<100ms",
      "<1 second",
      "<10 seconds"
    ]}
    correctIndex={1}
    explanation="Correct! Safe Physical AI requires <100ms control latency to react to dynamic environments (e.g., human proximity, obstacles). Cloud control is unsafe due to network latency."
    difficulty="hard"
  />
  EOF
  ```
  **Time**: 30 min
  **Acceptance**: Chapter 1 exists with 4 sections (Objectives, Theory, Lab, Assessment), 5 MCQs

- [ ] **T010** [P1] [P] Create Chapter 2: ROS 2 Fundamentals (template for remaining chapters)
  ```bash
  cd docusaurus
  mkdir -p docs/ch02-ros2-fundamentals
  cat > docs/ch02-ros2-fundamentals/index.md << 'EOF'
  ---
  id: ch02
  title: "ROS 2 Fundamentals"
  sidebar_position: 3
  week: 2
  objectives:
    - "Understand ROS 2 node architecture and communication patterns"
    - "Implement publisher-subscriber pattern with Python rclpy library"
    - "Debug ROS 2 topics using CLI tools (ros2 topic echo, ros2 node info)"
  tags: [ros2, fundamentals, nodes, topics, services]
  description: "Master ROS 2 core concepts: nodes, topics, services, and actions for robot control."
  ---

  # ROS 2 Fundamentals

  ## Objectives

  [Learning outcomes listed above in frontmatter]

  ---

  ## Theory

  ### ROS 2 Architecture

  [Content: Explain nodes, topics, services, actions with diagrams]

  ```mermaid
  graph LR
      A[Publisher Node] -->|/cmd_vel topic| B[Subscriber Node]
      B --> C[Robot Actuators]
  ```

  ### Communication Patterns

  [Content: Publisher-subscriber, service-client, action-goal patterns]

  ---

  ## Hands-on Lab

  ### Prerequisites

  - **Software**: ROS 2 Humble on Ubuntu 22.04
  - **Hardware (deployment)**: Jetson Orin Nano (see [Hardware Requirements](/hardware-requirements))

  ### Lab Code

  [Provide executable Python code for ROS 2 publisher-subscriber example]

  ```python
  # publisher_node.py
  import rclpy
  from rclpy.node import Node
  from std_msgs.msg import String

  class PublisherNode(Node):
      def __init__(self):
          super().__init__('publisher_node')
          self.publisher = self.create_publisher(String, '/my_topic', 10)
          self.timer = self.create_timer(1.0, self.publish_message)

      def publish_message(self):
          msg = String()
          msg.data = 'Hello ROS 2!'
          self.publisher.publish(msg)
          self.get_logger().info(f'Published: {msg.data}')

  def main(args=None):
      rclpy.init(args=args)
      node = PublisherNode()
      rclpy.spin(node)
      rclpy.shutdown()

  if __name__ == '__main__':
      main()
  ```

  **Run**:
  ```bash
  ros2 run my_package publisher_node
  ```

  ---

  ## Assessment

  import MCQ from '@site/src/components/MCQ';

  <MCQ
    id="ch02-mcq-01"
    question="What is the default middleware in ROS 2 Humble?"
    options={["DDS-RTPS", "ZeroMQ", "TCP/IP", "USB"]}
    correctIndex={0}
    explanation="Correct! ROS 2 Humble uses DDS-RTPS (Data Distribution Service) as default middleware. See [ROS 2 DDS Docs](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Different-Middleware-Vendors.html)."
    difficulty="medium"
  />

  [Add 4-9 more MCQs to total 5-10 per chapter]
  EOF
  ```
  **Time**: 30 min
  **Acceptance**: Chapter 2 template created, reusable for chapters 3-13

- [ ] **T011** [P2] [P] Create Chapters 3-13 using template from T010
  ```bash
  cd docusaurus

  # Chapter 3: Robot Modeling
  mkdir -p docs/ch03-robot-modeling
  cp docs/ch02-ros2-fundamentals/index.md docs/ch03-robot-modeling/index.md
  # Edit frontmatter: id: ch03, title: "Robot Modeling (URDF, TF2, Xacro)", sidebar_position: 4, week: 3

  # Chapter 4: Gazebo Simulation
  mkdir -p docs/ch04-gazebo-simulation
  cp docs/ch02-ros2-fundamentals/index.md docs/ch04-gazebo-simulation/index.md
  # Edit frontmatter: id: ch04, title: "Gazebo Simulation", sidebar_position: 5, week: 4

  # Chapter 5: Unity Simulation
  mkdir -p docs/ch05-unity-simulation
  cp docs/ch02-ros2-fundamentals/index.md docs/ch05-unity-simulation/index.md
  # Edit frontmatter: id: ch05, title: "Unity Simulation", sidebar_position: 6, week: 5

  # Chapter 6: NVIDIA Isaac Sim
  mkdir -p docs/ch06-isaac-sim
  cp docs/ch02-ros2-fundamentals/index.md docs/ch06-isaac-sim/index.md
  # Edit frontmatter: id: ch06, title: "NVIDIA Isaac Sim", sidebar_position: 7, week: 6

  # Chapter 7: Vision-Language-Action Models
  mkdir -p docs/ch07-vla-models
  cp docs/ch02-ros2-fundamentals/index.md docs/ch07-vla-models/index.md
  # Edit frontmatter: id: ch07, title: "Vision-Language-Action Models", sidebar_position: 8, week: 7

  # Chapter 8: Humanoid Kinematics
  mkdir -p docs/ch08-humanoid-kinematics
  cp docs/ch02-ros2-fundamentals/index.md docs/ch08-humanoid-kinematics/index.md
  # Edit frontmatter: id: ch08, title: "Humanoid Kinematics", sidebar_position: 9, week: 8

  # Chapter 9: Locomotion
  mkdir -p docs/ch09-locomotion
  cp docs/ch02-ros2-fundamentals/index.md docs/ch09-locomotion/index.md
  # Edit frontmatter: id: ch09, title: "Locomotion", sidebar_position: 10, week: 9

  # Chapter 10: Manipulation
  mkdir -p docs/ch10-manipulation
  cp docs/ch02-ros2-fundamentals/index.md docs/ch10-manipulation/index.md
  # Edit frontmatter: id: ch10, title: "Manipulation", sidebar_position: 11, week: 10

  # Chapter 11: Conversational AI Integration
  mkdir -p docs/ch11-conversational-ai
  cp docs/ch02-ros2-fundamentals/index.md docs/ch11-conversational-ai/index.md
  # Edit frontmatter: id: ch11, title: "Conversational AI Integration", sidebar_position: 12, week: 11

  # Chapter 12: Hardware Integration
  mkdir -p docs/ch12-hardware-integration
  cp docs/ch02-ros2-fundamentals/index.md docs/ch12-hardware-integration/index.md
  # Edit frontmatter: id: ch12, title: "Hardware Integration", sidebar_position: 13, week: 12

  # Chapter 13: Capstone Project
  mkdir -p docs/ch13-capstone-project
  cp docs/ch02-ros2-fundamentals/index.md docs/ch13-capstone-project/index.md
  # Edit frontmatter: id: ch13, title: "Capstone Project", sidebar_position: 14, week: 13
  ```
  **Time**: 90 min (edit frontmatter + placeholder content for each)
  **Acceptance**: 13 chapter folders exist, each with index.md containing frontmatter + 4-section structure

**Checkpoint**: All 13 chapters scaffolded with MDX templates, ready for content writing (Phase 3)

---

## Phase 3: React Components [P2] 🎯 Core Features

**Purpose**: Build MCQ component, collapsible lab sections, landing page animations

**Estimated Time**: 4-5 hours

- [ ] **T012** [P2] [P] Create MCQ React component with instant feedback
  ```bash
  cd docusaurus
  mkdir -p src/components/MCQ
  cat > src/components/MCQ/index.tsx << 'EOF'
  import React, { useState } from 'react';
  import styles from './MCQ.module.css';

  interface MCQProps {
    id: string;
    question: string;
    options: string[];
    correctIndex: number;
    explanation: string;
    difficulty?: 'easy' | 'medium' | 'hard';
  }

  export default function MCQ({
    id,
    question,
    options,
    correctIndex,
    explanation,
    difficulty = 'medium',
  }: MCQProps): JSX.Element {
    const [selected, setSelected] = useState<number | null>(null);
    const [showFeedback, setShowFeedback] = useState(false);

    const handleSubmit = () => {
      if (selected !== null) {
        setShowFeedback(true);
        // Optional: Save to localStorage
        const progress = JSON.parse(localStorage.getItem('physicalai_user_progress') || '{}');
        if (!progress.chapters) progress.chapters = {};
        const chapterId = id.split('-')[0]; // Extract "ch01" from "ch01-mcq-01"
        if (!progress.chapters[chapterId]) progress.chapters[chapterId] = { mcqScores: {} };
        progress.chapters[chapterId].mcqScores[id] = {
          selectedIndex: selected,
          correct: selected === correctIndex,
          attemptedAt: new Date().toISOString(),
        };
        progress.lastUpdated = new Date().toISOString();
        localStorage.setItem('physicalai_user_progress', JSON.stringify(progress));
      }
    };

    const isCorrect = selected === correctIndex;

    return (
      <div className={styles.mcqContainer} data-difficulty={difficulty}>
        <h3 className={styles.question}>{question}</h3>
        <div className={styles.optionsGroup} role="radiogroup" aria-labelledby={`question-${id}`}>
          {options.map((option, idx) => {
            const isSelected = selected === idx;
            const isCorrectOption = idx === correctIndex;
            const showCorrect = showFeedback && isCorrectOption;
            const showIncorrect = showFeedback && isSelected && !isCorrectOption;

            return (
              <label
                key={idx}
                className={`${styles.option} ${showCorrect ? styles.correct : ''} ${showIncorrect ? styles.incorrect : ''}`}
              >
                <input
                  type="radio"
                  name={`mcq-${id}`}
                  checked={isSelected}
                  onChange={() => setSelected(idx)}
                  disabled={showFeedback}
                />
                <span className={styles.optionLabel}>
                  {String.fromCharCode(65 + idx)}. {option}
                </span>
              </label>
            );
          })}
        </div>
        <button
          className={styles.submitButton}
          onClick={handleSubmit}
          disabled={selected === null || showFeedback}
        >
          {showFeedback ? 'Submitted' : 'Submit Answer'}
        </button>
        {showFeedback && (
          <div className={`${styles.feedback} ${isCorrect ? styles.feedbackCorrect : styles.feedbackIncorrect}`}>
            <p className={styles.feedbackIcon}>{isCorrect ? '✅ Correct!' : '❌ Incorrect'}</p>
            <p className={styles.explanation}>{explanation}</p>
          </div>
        )}
      </div>
    );
  }
  EOF
  ```
  **Time**: 60 min
  **Acceptance**: MCQ component renders question, 4 options, submit button, instant feedback

- [ ] **T013** [P2] [P] Create MCQ component styles (dark mode compatible)
  ```bash
  cd docusaurus
  cat > src/components/MCQ/MCQ.module.css << 'EOF'
  .mcqContainer {
    margin: 2rem 0;
    padding: 1.5rem;
    border: 1px solid var(--ifm-color-emphasis-300);
    border-radius: 8px;
    background: var(--ifm-background-surface-color);
  }

  .question {
    margin-top: 0;
    font-size: 1.1rem;
    color: var(--ifm-font-color-base);
  }

  .optionsGroup {
    margin: 1rem 0;
  }

  .option {
    display: flex;
    align-items: center;
    padding: 0.75rem;
    margin: 0.5rem 0;
    border: 2px solid var(--ifm-color-emphasis-300);
    border-radius: 6px;
    cursor: pointer;
    transition: all 0.2s ease;
  }

  .option:hover {
    background: var(--ifm-color-emphasis-100);
  }

  .option input[type="radio"] {
    margin-right: 0.75rem;
  }

  .optionLabel {
    flex: 1;
  }

  .correct {
    background: var(--ifm-color-success-lightest);
    border-color: var(--ifm-color-success);
  }

  .incorrect {
    background: var(--ifm-color-danger-lightest);
    border-color: var(--ifm-color-danger);
  }

  .submitButton {
    padding: 0.5rem 1.5rem;
    background: var(--ifm-color-primary);
    color: white;
    border: none;
    border-radius: 4px;
    cursor: pointer;
    font-size: 1rem;
    transition: background 0.2s;
  }

  .submitButton:hover:not(:disabled) {
    background: var(--ifm-color-primary-dark);
  }

  .submitButton:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }

  .feedback {
    margin-top: 1rem;
    padding: 1rem;
    border-radius: 6px;
  }

  .feedbackCorrect {
    background: var(--ifm-color-success-lightest);
    border-left: 4px solid var(--ifm-color-success);
  }

  .feedbackIncorrect {
    background: var(--ifm-color-danger-lightest);
    border-left: 4px solid var(--ifm-color-danger);
  }

  .feedbackIcon {
    font-size: 1.2rem;
    font-weight: bold;
    margin: 0 0 0.5rem 0;
  }

  .explanation {
    margin: 0;
    line-height: 1.6;
  }
  EOF
  ```
  **Time**: 20 min
  **Acceptance**: MCQ styled with green/red feedback, dark mode variables used

- [ ] **T014** [P2] [P] Create MCQ component tests (Jest + React Testing Library)
  ```bash
  cd docusaurus
  cat > src/components/MCQ/MCQ.test.tsx << 'EOF'
  import React from 'react';
  import { render, screen, fireEvent } from '@testing-library/react';
  import MCQ from './index';

  const mockProps = {
    id: 'test-mcq-01',
    question: 'What is 2 + 2?',
    options: ['3', '4', '5', '6'],
    correctIndex: 1,
    explanation: 'Correct! 2 + 2 = 4.',
  };

  test('renders MCQ with question and options', () => {
    render(<MCQ {...mockProps} />);
    expect(screen.getByText('What is 2 + 2?')).toBeInTheDocument();
    expect(screen.getByLabelText(/A\. 3/)).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /submit answer/i })).toBeDisabled();
  });

  test('enables submit button when option selected', () => {
    render(<MCQ {...mockProps} />);
    const optionB = screen.getByLabelText(/B\. 4/);
    fireEvent.click(optionB);
    expect(screen.getByRole('button', { name: /submit answer/i })).toBeEnabled();
  });

  test('shows correct feedback after submission', () => {
    render(<MCQ {...mockProps} />);
    fireEvent.click(screen.getByLabelText(/B\. 4/));
    fireEvent.click(screen.getByRole('button', { name: /submit answer/i }));
    expect(screen.getByText(/✅ correct/i)).toBeInTheDocument();
    expect(screen.getByText('Correct! 2 + 2 = 4.')).toBeInTheDocument();
  });

  test('shows incorrect feedback when wrong option selected', () => {
    render(<MCQ {...mockProps} />);
    fireEvent.click(screen.getByLabelText(/A\. 3/));
    fireEvent.click(screen.getByRole('button', { name: /submit answer/i }));
    expect(screen.getByText(/❌ incorrect/i)).toBeInTheDocument();
  });
  EOF
  ```
  **Time**: 30 min
  **Acceptance**: Tests pass with `npm run test`, coverage >80%

- [ ] **T015** [P2] [P] Create CollapsibleLab component for code sections
  ```bash
  cd docusaurus
  mkdir -p src/components/CollapsibleLab
  cat > src/components/CollapsibleLab/index.tsx << 'EOF'
  import React, { useState } from 'react';
  import styles from './CollapsibleLab.module.css';

  interface CollapsibleLabProps {
    title: string;
    children: React.ReactNode;
    defaultOpen?: boolean;
  }

  export default function CollapsibleLab({
    title,
    children,
    defaultOpen = false,
  }: CollapsibleLabProps): JSX.Element {
    const [isOpen, setIsOpen] = useState(defaultOpen);

    return (
      <div className={styles.collapsibleLab}>
        <button
          className={styles.toggleButton}
          onClick={() => setIsOpen(!isOpen)}
          aria-expanded={isOpen}
        >
          <span className={styles.icon}>{isOpen ? '▼' : '▶'}</span>
          <span className={styles.title}>{title}</span>
        </button>
        {isOpen && <div className={styles.content}>{children}</div>}
      </div>
    );
  }
  EOF

  cat > src/components/CollapsibleLab/CollapsibleLab.module.css << 'EOF'
  .collapsibleLab {
    margin: 1rem 0;
    border: 1px solid var(--ifm-color-emphasis-300);
    border-radius: 6px;
    overflow: hidden;
  }

  .toggleButton {
    width: 100%;
    display: flex;
    align-items: center;
    padding: 0.75rem 1rem;
    background: var(--ifm-color-emphasis-100);
    border: none;
    cursor: pointer;
    font-size: 1rem;
    text-align: left;
    transition: background 0.2s;
  }

  .toggleButton:hover {
    background: var(--ifm-color-emphasis-200);
  }

  .icon {
    margin-right: 0.5rem;
    font-size: 0.8rem;
  }

  .title {
    font-weight: 600;
  }

  .content {
    padding: 1rem;
  }
  EOF
  ```
  **Time**: 30 min
  **Acceptance**: CollapsibleLab component toggles visibility, styled for dark mode

- [ ] **T016** [P3] [P] Create custom landing page with animations (React Spring or CSS)
  ```bash
  cd docusaurus
  cat > src/pages/index.tsx << 'EOF'
  import React from 'react';
  import Layout from '@theme/Layout';
  import Link from '@docusaurus/Link';
  import styles from './index.module.css';

  export default function Home(): JSX.Element {
    return (
      <Layout
        title="Welcome"
        description="Physical AI & Humanoid Robotics Textbook"
      >
        <header className={styles.heroBanner}>
          <div className="container">
            <h1 className={styles.heroTitle}>
              Physical AI & Humanoid Robotics
            </h1>
            <p className={styles.heroSubtitle}>
              Learn to build intelligent humanoid robots with ROS 2, Isaac Sim, and VLA Models
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro"
              >
                Start Learning →
              </Link>
              <Link
                className="button button--secondary button--lg"
                to="/docs/hardware-requirements"
                style={{ marginLeft: '1rem' }}
              >
                View Hardware Options
              </Link>
            </div>
          </div>
        </header>

        <main>
          <section className={styles.features}>
            <div className="container">
              <div className="row">
                <div className="col col--4">
                  <div className={styles.feature}>
                    <h3>🤖 Hands-On Learning</h3>
                    <p>Executable code for ROS 2, Gazebo, Isaac Sim - simulation to hardware deployment</p>
                  </div>
                </div>
                <div className="col col--4">
                  <div className={styles.feature}>
                    <h3>📚 13 Comprehensive Chapters</h3>
                    <p>From ROS 2 basics to VLA models and capstone humanoid project</p>
                  </div>
                </div>
                <div className="col col--4">
                  <div className={styles.feature}>
                    <h3>💰 Budget-Friendly Options</h3>
                    <p>Three tiers: Economy ($700), Proxy ($3k), Cloud ($205/quarter)</p>
                  </div>
                </div>
              </div>
            </div>
          </section>
        </main>
      </Layout>
    );
  }
  EOF

  cat > src/pages/index.module.css << 'EOF'
  .heroBanner {
    padding: 4rem 0;
    text-align: center;
    position: relative;
    overflow: hidden;
    background: linear-gradient(135deg, var(--ifm-color-primary-lightest) 0%, var(--ifm-color-primary-light) 100%);
  }

  .heroTitle {
    font-size: 3rem;
    margin-bottom: 1rem;
    animation: fadeInUp 0.6s ease-out;
  }

  .heroSubtitle {
    font-size: 1.5rem;
    margin-bottom: 2rem;
    opacity: 0.9;
    animation: fadeInUp 0.6s ease-out 0.2s backwards;
  }

  .buttons {
    animation: fadeInUp 0.6s ease-out 0.4s backwards;
  }

  @keyframes fadeInUp {
    from {
      opacity: 0;
      transform: translateY(20px);
    }
    to {
      opacity: 1;
      transform: translateY(0);
    }
  }

  .features {
    padding: 4rem 0;
  }

  .feature {
    text-align: center;
    padding: 2rem;
  }

  .feature h3 {
    font-size: 1.5rem;
    margin-bottom: 1rem;
  }
  EOF
  ```
  **Time**: 45 min
  **Acceptance**: Custom landing page with fade-in animations, links to chapters and hardware

**Checkpoint**: All React components built (MCQ, CollapsibleLab, Landing) and tested

---

## Phase 4: Hardware Requirements Page [P3] 🎯 Enhancement

**Purpose**: Create hardware page with three budget tiers and pricing tables

**Estimated Time**: 2-3 hours

- [ ] **T017** [P3] Create hardware-tiers.yaml data file
  ```bash
  cd docusaurus
  mkdir -p static/data
  cat > static/data/hardware-tiers.yaml << 'EOF'
  tiers:
    - name: Economy
      totalCost: 700
      currency: USD
      components:
        - name: Jetson Orin Nano Developer Kit
          price: 249
          vendor: NVIDIA
          link: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/
          specs: 8GB RAM, 40 TOPS AI performance
        - name: Intel RealSense D435i
          price: 349
          vendor: Intel
          link: https://www.intelrealsense.com/depth-camera-d435i/
          specs: Depth + RGB, IMU, USB 3.1
        - name: Accessories (Power, microSD, Cooling)
          price: 102
          vendor: Generic
          link: https://www.amazon.com/
          specs: 15V 4A PSU, 128GB microSD, cooling fan
      useCases:
        - Solo learners with budget constraints
        - DIY enthusiasts building personal projects
        - Students without institutional lab access
      pros:
        - Low upfront cost (~$700 one-time)
        - Portable and self-contained
        - Sufficient for ROS 2 basics and edge deployment
      cons:
        - Limited to edge computing (cannot run Isaac Sim locally)
        - Jetson less powerful than desktop RTX GPU

    - name: Proxy
      totalCost: 3850
      currency: USD
      components:
        - name: Unitree Go2 Quadruped Robot
          price: 3000
          vendor: Unitree
          link: https://www.unitree.com/go2/
          specs: 12 DOF, 25kg payload, LiDAR, RealSense
        - name: Jetson Xavier NX
          price: 500
          vendor: NVIDIA
          link: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-xavier-nx/
          specs: 21 TOPS, 8GB/16GB RAM
        - name: Accessories
          price: 350
          vendor: Generic
          link: https://www.amazon.com/
          specs: Backup RealSense D435i, cables, battery
      useCases:
        - Research labs with locomotion focus
        - University courses with physical robot demos
        - Proxy for expensive humanoid robots
      pros:
        - Real robot hardware (sim-to-real validation)
        - Unitree Go2 mimics humanoid locomotion patterns
        - Scalable to Unitree G1 humanoid ($16k) later
      cons:
        - High upfront cost (~$3850)
        - Requires lab space (2m x 2m clear area)

    - name: Cloud OpEx
      totalCost: 205
      currency: USD
      period: per quarter (3 months)
      components:
        - name: AWS EC2 g4dn.xlarge instance
          price: 192
          vendor: AWS
          link: https://aws.amazon.com/ec2/instance-types/g4/
          specs: 4 vCPUs, 16GB RAM, NVIDIA T4 GPU
          calculation: $1.20/hr × 40 hours/quarter
        - name: AWS EBS Storage (100GB SSD)
          price: 10
          vendor: AWS
          link: https://aws.amazon.com/ebs/pricing/
          specs: gp3 SSD, 100GB
        - name: Data Transfer
          price: 3
          vendor: AWS
          link: https://aws.amazon.com/ec2/pricing/
          specs: ~30GB/quarter outbound
      useCases:
        - Institutions without CapEx budgets
        - Students with cloud credits (AWS Educate)
        - Pay-as-you-go model (pause when not in use)
      pros:
        - No upfront hardware cost (OpEx only)
        - Scalable GPU power
        - Access to Isaac Sim with T4 GPU
      cons:
        - Recurring cost ($205/quarter = $820/year)
        - Requires stable internet
  EOF
  ```
  **Time**: 30 min
  **Acceptance**: hardware-tiers.yaml created with 3 tiers (Economy, Proxy, Cloud) and accurate pricing

- [ ] **T018** [P3] Create hardware-requirements.md page with pricing tables
  ```bash
  cd docusaurus
  cat > docs/hardware-requirements.md << 'EOF'
  ---
  id: hardware-requirements
  title: "Hardware Requirements"
  sidebar_position: 15
  description: "Three budget tiers for Physical AI learning: Economy ($700), Proxy ($3k), Cloud ($205/quarter)"
  ---

  # Hardware Requirements

  Choose a hardware tier based on your budget, learning goals, and access to physical robots.

  ## Overview

  | Tier       | Total Cost      | Best For                          | Key Benefit               |
  |------------|-----------------|-----------------------------------|---------------------------|
  | Economy    | ~$700 (one-time)| Solo learners, students           | Low upfront cost          |
  | Proxy      | ~$3850 (one-time)| Research labs, institutions      | Real robot hardware       |
  | Cloud OpEx | ~$205/quarter   | Cloud-only learners, institutions | No hardware maintenance   |

  ---

  ## Economy Tier: Jetson Edge Kit (~$700)

  **Target Audience**: Solo learners, DIY enthusiasts, students without institutional lab access

  ### Components

  | Component                     | Price | Vendor | Specs                              | Link                                      |
  |-------------------------------|-------|--------|------------------------------------|-------------------------------------------|
  | Jetson Orin Nano Developer Kit| $249  | NVIDIA | 8GB RAM, 40 TOPS AI                | [NVIDIA Store](https://www.nvidia.com/)   |
  | Intel RealSense D435i         | $349  | Intel  | Depth + RGB, IMU                   | [Intel RealSense](https://www.intelrealsense.com/) |
  | Accessories (PSU, microSD)    | $102  | Generic| 15V 4A, 128GB, cooling fan         | [Amazon](https://www.amazon.com/)         |
  | **Total**                     | **$700** |     |                                    |                                           |

  ### Use Cases

  - Run ROS 2 nodes locally on Jetson
  - Deploy trained models from Isaac Sim to edge hardware
  - Integrate RealSense for depth perception and SLAM
  - Cannot run Isaac Sim locally (use cloud VM or remote desktop)

  ### Pros & Cons

  ✅ **Pros**:
  - Low upfront cost (~$700 one-time investment)
  - Portable and self-contained (no external workstation needed)
  - Sufficient for ROS 2 basics and edge deployment
  - Can run Isaac Sim remotely (AWS EC2 g4dn.xlarge)

  ❌ **Cons**:
  - Limited to edge computing (Jetson cannot run Isaac Sim locally)
  - Jetson Orin Nano less powerful than desktop RTX GPU
  - Requires additional peripherals (monitor, keyboard, mouse for initial setup)

  ---

  ## Proxy Tier: Unitree Go2 + Jetson (~$3850)

  **Target Audience**: Research labs, university courses, humanoid locomotion research

  ### Components

  | Component                | Price  | Vendor  | Specs                              | Link                                |
  |--------------------------|--------|---------|------------------------------------|------------------------------------|
  | Unitree Go2 Quadruped    | $3000  | Unitree | 12 DOF, 25kg payload, LiDAR        | [Unitree](https://www.unitree.com/)|
  | Jetson Xavier NX         | $500   | NVIDIA  | 21 TOPS, 8GB/16GB RAM              | [NVIDIA](https://www.nvidia.com/)  |
  | Accessories              | $350   | Generic | Backup RealSense, cables, battery  | [Amazon](https://www.amazon.com/)  |
  | **Total**                | **$3850** |      |                                    |                                    |

  ### Use Cases

  - Validate humanoid locomotion algorithms on real quadruped (proxy for bipedal)
  - Test perception pipelines (LiDAR + RealSense fusion)
  - Whole-body control experiments
  - Sim-to-real transfer validation (Isaac Sim → Unitree Go2)

  ### Pros & Cons

  ✅ **Pros**:
  - Real robot hardware (sim-to-real validation)
  - Unitree Go2 mimics humanoid locomotion patterns (12 DOF legs)
  - Included sensors (LiDAR, IMU, cameras)
  - Scalable to Unitree G1 humanoid ($16k) later

  ❌ **Cons**:
  - High upfront cost (~$3850)
  - Requires lab space (2m x 2m clear area)
  - Maintenance and wear-and-tear costs
  - Unitree G1 not included (Go2 is proxy only)

  ---

  ## Cloud OpEx Tier: AWS EC2 (~$205/quarter)

  **Target Audience**: Institutions without CapEx budgets, students with cloud credits, remote learners

  ### Components (Per Quarter)

  | Service                    | Cost/Quarter | Calculation                      | Specs                    | Link                         |
  |----------------------------|--------------|----------------------------------|--------------------------|------------------------------|
  | AWS EC2 g4dn.xlarge        | $192         | $1.20/hr × 40 hours/quarter      | 4 vCPUs, 16GB, T4 GPU    | [AWS EC2](https://aws.amazon.com/ec2/) |
  | AWS EBS Storage (100GB)    | $10          | $0.08/GB/month × 100GB × 3 months| gp3 SSD                  | [AWS EBS](https://aws.amazon.com/ebs/) |
  | Data Transfer (outbound)   | $3           | $0.09/GB × 30GB/quarter          | Sim datasets, downloads  | [AWS Pricing](https://aws.amazon.com/) |
  | **Total**                  | **$205/quarter** |                              | **$820/year**            |                              |

  ### Use Cases

  - Run Isaac Sim on cloud GPU (T4 adequate for labs)
  - Access from any device (Windows, Mac, Linux)
  - Scale GPU power as needed (upgrade to g4dn.2xlarge for 2× T4)
  - Pause instance when not in use (pay-as-you-go)

  ### Pros & Cons

  ✅ **Pros**:
  - No upfront hardware cost (OpEx only)
  - Scalable GPU power (T4, A10, A100 options)
  - Automatic backups and snapshots (EBS)
  - Access to Isaac Sim without local RTX GPU

  ❌ **Cons**:
  - Recurring quarterly cost ($205/quarter = $820/year)
  - Requires stable internet (latency for remote desktop)
  - AWS free tier limited (12 months, then pay)
  - Cannot deploy to physical Jetson (need Economy tier for that)

  ---

  ## Decision Guide

  ### Choose **Economy Tier** if:
  - You want hands-on hardware experience
  - Budget is limited (~$700 one-time)
  - You plan to deploy to Jetson long-term

  ### Choose **Proxy Tier** if:
  - You need real robot validation
  - Institution has lab space and budget
  - Focus on locomotion/whole-body control

  ### Choose **Cloud OpEx Tier** if:
  - No upfront CapEx budget available
  - You have cloud credits (AWS Educate, GitHub Student Pack)
  - Remote learning is priority (no local GPU)

  ---

  ## Last Updated

  **Date**: 2025-12-09

  **Pricing Sources**:
  - NVIDIA Jetson: [https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
  - Intel RealSense: [https://www.intelrealsense.com/depth-camera-d435i/](https://www.intelrealsense.com/depth-camera-d435i/)
  - Unitree Robotics: [https://www.unitree.com/go2/](https://www.unitree.com/go2/)
  - AWS Pricing: [https://aws.amazon.com/ec2/pricing/](https://aws.amazon.com/ec2/pricing/)

  **Disclaimer**: Prices subject to change. Verify with vendors before purchase.
  EOF
  ```
  **Time**: 60 min
  **Acceptance**: Hardware Requirements page with 3 tiers, pricing tables, decision guide

**Checkpoint**: Hardware page complete with accurate pricing and purchase links

---

## Phase 5: Static Assets & Styling [P3] 🎯 Enhancement

**Purpose**: Add custom CSS, images, data files

**Estimated Time**: 1-2 hours

- [ ] **T019** [P3] [P] Add custom CSS for dark mode and mobile responsiveness
  ```bash
  cd docusaurus
  cat >> src/css/custom.css << 'EOF'
  /* Custom CSS for Physical AI Textbook */

  :root {
    --ifm-color-primary: #2e8555;
    --ifm-color-primary-dark: #29784c;
    --ifm-color-primary-darker: #277148;
    --ifm-color-primary-darkest: #205d3b;
    --ifm-color-primary-light: #33925d;
    --ifm-color-primary-lighter: #359962;
    --ifm-color-primary-lightest: #3cad6e;
    --ifm-code-font-size: 95%;
  }

  [data-theme='dark'] {
    --ifm-color-primary: #25c2a0;
    --ifm-color-primary-dark: #21af90;
    --ifm-color-primary-darker: #1fa588;
    --ifm-color-primary-darkest: #1a8870;
    --ifm-color-primary-light: #29d5b0;
    --ifm-color-primary-lighter: #32d8b4;
    --ifm-color-primary-lightest: #4fddbf;
  }

  /* Mobile responsiveness */
  @media screen and (max-width: 768px) {
    .heroBanner {
      padding: 2rem 0;
    }

    .heroTitle {
      font-size: 2rem;
    }

    .heroSubtitle {
      font-size: 1.2rem;
    }
  }

  /* Code block enhancements */
  pre {
    border-radius: 6px;
  }

  /* MCQ container spacing */
  .mcqContainer {
    box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
  }

  [data-theme='dark'] .mcqContainer {
    box-shadow: 0 2px 8px rgba(255, 255, 255, 0.1);
  }

  /* Table responsiveness */
  table {
    display: block;
    overflow-x: auto;
    white-space: nowrap;
  }

  @media screen and (max-width: 768px) {
    table {
      font-size: 0.9rem;
    }
  }
  EOF
  ```
  **Time**: 20 min
  **Acceptance**: Custom CSS applied, dark mode colors updated, mobile responsive

- [ ] **T020** [P3] [P] Add placeholder images and logo
  ```bash
  cd docusaurus
  # Add placeholder logo (replace with actual logo later)
  echo "<!-- Placeholder: Add robot logo SVG here -->" > static/img/logo.svg

  # Add favicon
  echo "<!-- Placeholder: Add favicon.ico here -->" > static/img/favicon.ico

  # Create img directory structure
  mkdir -p static/img/{robots,diagrams,hardware}
  ```
  **Time**: 10 min
  **Acceptance**: Image directories created, placeholders for logo/favicon

- [ ] **T021** [P3] Create README.md with setup instructions
  ```bash
  cd ..
  cat > README.md << 'EOF'
  # Physical AI & Humanoid Robotics Textbook

  Comprehensive interactive textbook teaching ROS 2, Gazebo/Unity simulation, NVIDIA Isaac Sim, and Vision-Language-Action models for humanoid robots.

  ## Features

  - **13 Chapters**: Week-by-week progression from ROS 2 basics to capstone project
  - **Interactive MCQs**: Instant feedback with explanations
  - **Hands-on Labs**: Executable code for simulation and hardware deployment
  - **Hardware Guides**: Three budget tiers (Economy $700, Proxy $3k, Cloud $205/quarter)
  - **Dark Mode**: Fully responsive with light/dark theme toggle
  - **i18n Ready**: Stub for future Urdu translation

  ## Quick Start

  ### Prerequisites

  - Node.js 20.x LTS ([Download](https://nodejs.org/))
  - Git

  ### Local Development

  ```bash
  git clone https://github.com/<org>/physical-ai-textbook.git
  cd physical-ai-textbook/docusaurus
  npm install
  npm run start
  ```

  Open [http://localhost:3000/](http://localhost:3000/) to view the site.

  ### Build for Production

  ```bash
  cd docusaurus
  npm run build
  npm run serve
  ```

  ### Deploy to GitHub Pages

  1. Enable GitHub Pages in repo settings (Settings > Pages > Source: "GitHub Actions")
  2. Configure `docusaurus.config.js` with your repo URL
  3. Push to `main` branch - GitHub Actions will automatically deploy

  ## Project Structure

  ```
  physical-ai-textbook/
  ├── docusaurus/              # Docusaurus site
  │   ├── docs/                # 13 chapters + hardware page
  │   ├── src/                 # React components (MCQ, CollapsibleLab)
  │   ├── static/              # Images, data files
  │   └── docusaurus.config.js # Configuration
  ├── .github/workflows/       # CI/CD pipeline
  └── README.md                # This file
  ```

  ## Technology Stack

  - **Framework**: Docusaurus 3.x (React 18, MDX 3)
  - **Diagrams**: Mermaid (via @docusaurus/theme-mermaid)
  - **CI/CD**: GitHub Actions
  - **Hosting**: GitHub Pages (primary), Vercel (fallback)

  ## Contributing

  See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

  ## License

  - **Content**: Creative Commons BY-SA 4.0
  - **Code**: MIT License

  ## Support

  - **Issues**: [GitHub Issues](https://github.com/<org>/physical-ai-textbook/issues)
  - **Discussions**: [GitHub Discussions](https://github.com/<org>/physical-ai-textbook/discussions)
  EOF
  ```
  **Time**: 20 min
  **Acceptance**: README.md created with setup, structure, tech stack docs

**Checkpoint**: Static assets, styling, and documentation complete

---

## Phase 6: Testing & Validation [P2] 🎯 Quality Assurance

**Purpose**: Run tests, validate build, check accessibility

**Estimated Time**: 1-2 hours

- [ ] **T022** [P2] Run MCQ component tests
  ```bash
  cd docusaurus
  npm run test
  ```
  **Time**: 5 min
  **Acceptance**: All tests pass (MCQ.test.tsx), coverage >80%

- [ ] **T023** [P2] Build site and check for errors
  ```bash
  cd docusaurus
  npm run build
  ```
  **Time**: 3 min
  **Acceptance**: Build succeeds with 0 errors, warnings acceptable, build/ folder created

- [ ] **T024** [P2] Serve built site locally and manual test
  ```bash
  cd docusaurus
  npm run serve
  # Open http://localhost:3000/
  # Test:
  # - Navigate to Chapter 1, verify MCQ renders
  # - Toggle dark mode (moon icon in navbar)
  # - Test mobile view (browser DevTools)
  # - Check Mermaid diagram renders (Chapter 1 Theory section)
  ```
  **Time**: 15 min
  **Acceptance**: Site navigable, MCQs functional, dark mode works, mobile responsive

- [ ] **T025** [P2] Run Lighthouse CI for performance/accessibility
  ```bash
  cd docusaurus
  npm install -g @lhci/cli
  lhci autorun --collect.staticDistDir=./build
  ```
  **Time**: 10 min
  **Acceptance**: Lighthouse scores >90 for Performance, Accessibility, Best Practices

- [ ] **T026** [P2] Validate Markdown links (no broken links)
  ```bash
  cd docusaurus
  npm install -g linkinator
  linkinator ./build --recurse --skip "http://localhost"
  ```
  **Time**: 5 min
  **Acceptance**: 0 broken links (404 errors)

**Checkpoint**: All tests pass, build succeeds, site meets quality standards

---

## Phase 7: Deployment & Documentation [P1] 🎯 Go Live

**Purpose**: Deploy to GitHub Pages, finalize documentation

**Estimated Time**: 1 hour

- [ ] **T027** [P1] Push code to GitHub repository
  ```bash
  cd ..
  git add .
  git commit -m "feat: complete Physical AI textbook Docusaurus site

  - Add Docusaurus 3.x scaffold with Next.js integration
  - Create 13 chapter MDX templates with frontmatter + 4-section structure
  - Implement MCQ React component with instant feedback (localStorage progress)
  - Add CollapsibleLab component for code sections
  - Create custom landing page with fade-in animations
  - Add Hardware Requirements page with 3 budget tiers (Economy, Proxy, Cloud)
  - Configure GitHub Actions workflow for Pages deployment
  - Add Mermaid diagram support (@docusaurus/theme-mermaid)
  - Setup i18n stub for Urdu translation (ur locale)
  - Add custom CSS for dark mode and mobile responsiveness
  - Include Lighthouse CI checks (Performance >90, Accessibility >90)

  Ready for deployment and content writing."
  git push origin 1-physical-ai-textbook
  ```
  **Time**: 5 min
  **Acceptance**: Code pushed to GitHub, branch `1-physical-ai-textbook` visible

- [ ] **T028** [P1] Enable GitHub Pages in repository settings
  ```text
  1. Go to GitHub repo > Settings > Pages
  2. Source: "GitHub Actions" (not "Deploy from branch")
  3. Custom domain (optional): physicalai.yourdomain.com
  4. Save
  ```
  **Time**: 5 min
  **Acceptance**: GitHub Pages enabled with Actions as source

- [ ] **T029** [P1] Merge feature branch to main (or create PR)
  ```bash
  git checkout main
  git merge 1-physical-ai-textbook
  git push origin main
  ```
  **Time**: 5 min
  **Acceptance**: Code merged to main branch, triggers GitHub Actions workflow

- [ ] **T030** [P1] Monitor GitHub Actions deployment
  ```text
  1. Go to GitHub repo > Actions tab
  2. Watch "Deploy Physical AI Textbook to GitHub Pages" workflow
  3. Check build logs for errors
  4. Wait for deployment to complete (~3-5 minutes)
  ```
  **Time**: 10 min
  **Acceptance**: Workflow succeeds, green checkmark, site deployed

- [ ] **T031** [P1] Verify deployed site
  ```text
  1. Open https://<username>.github.io/physical-ai-textbook/
  2. Test navigation (Chapter 1, Hardware Requirements)
  3. Test MCQ functionality (select answer, submit, see feedback)
  4. Test dark mode toggle
  5. Test mobile view (resize browser or use DevTools)
  ```
  **Time**: 10 min
  **Acceptance**: Site live, all features functional, accessible globally

- [ ] **T032** [P3] (Optional) Setup Vercel as fallback deployment
  ```bash
  # Install Vercel CLI
  npm i -g vercel

  # Login and deploy
  cd docusaurus
  vercel login
  vercel --prod
  ```
  **Time**: 15 min
  **Acceptance**: Vercel deployment live at https://physical-ai-textbook.vercel.app/ (or custom domain)

**Checkpoint**: Site deployed to production, accessible globally, all features working

---

## Phase 8: Content Writing (Future) [P2] 📝 Expand Chapters

**Purpose**: Write detailed Theory, Labs, and MCQs for all 13 chapters (Post-MVP)

**Note**: This phase is for future content creation after MVP deployment. Use T010 chapter template as starting point.

**Estimated Time**: 40-60 hours (3-5 hours per chapter × 13 chapters)

- [ ] **T033** [P2] Write Chapter 2 content (ROS 2 Fundamentals)
  - Theory: ROS 2 architecture, nodes, topics, services, actions (3000 words)
  - Lab: Publisher-subscriber example, service client, launch files (500 lines code)
  - Assessment: 7-10 MCQs covering ROS 2 concepts
  **Time**: 4 hours

- [ ] **T034** [P2] Write Chapter 3 content (Robot Modeling - URDF, TF2)
  **Time**: 4 hours

- [ ] **T035** [P2] Write Chapters 4-13 content (repeat for each)
  **Time**: 4 hours per chapter × 10 chapters = 40 hours

**Note**: Use Claude Code `BookWriterAgent` (constitution) to parallelize chapter writing. Generate outlines, then fill with citations and code examples.

---

## Dependencies & Execution Order

### Critical Path (Must Complete Sequentially)

1. **Phase 1**: T001 → T007 (Project setup, cannot start other phases without Docusaurus scaffold)
2. **Phase 2**: T008 → T011 (Chapter structure, depends on Phase 1 complete)
3. **Phase 6**: T022 → T026 (Testing, depends on code from Phases 1-5)
4. **Phase 7**: T027 → T031 (Deployment, depends on testing pass)

### Parallelizable Tasks

**After T007 (Docusaurus setup complete), run in parallel**:
- Phase 2 (Chapter templates): T008, T009, T010 can run concurrently (different files)
- Phase 3 (React components): T012, T015, T016 can run concurrently (MCQ, CollapsibleLab, Landing page)
- Phase 4 (Hardware page): T017, T018 independent of other phases
- Phase 5 (Static assets): T019, T020, T021 can run in parallel

**Optimal Parallel Execution**:
```
T001-T007 (Sequential)
    ↓
[T008 || T009 || T010] (Parallel: Chapter templates)
[T012 || T015 || T016] (Parallel: React components)
[T017 || T018] (Parallel: Hardware page)
[T019 || T020 || T021] (Parallel: Static assets)
    ↓
T011 (Sequential: Generate remaining chapters from template)
    ↓
T022-T026 (Sequential: Testing & validation)
    ↓
T027-T031 (Sequential: Deployment)
```

**Estimated Total Time (Single Developer)**:
- Sequential execution: ~20-25 hours
- Parallel execution (max concurrency): ~12-15 hours

---

## Validation Checklist (Before Marking Complete)

### Phase 1: Project Setup ✅
- [ ] Docusaurus project created at `docusaurus/` folder
- [ ] `npm install` succeeds, `npm run start` serves site at localhost:3000
- [ ] `docusaurus.config.js` configured (Mermaid, i18n, metadata)
- [ ] `sidebars.js` lists 13 chapters + hardware page
- [ ] `.github/workflows/deploy.yml` exists and passes validation
- [ ] i18n/ur/ directory created with empty code.json

### Phase 2: Chapter Structure ✅
- [ ] `docs/intro.md` landing page exists
- [ ] 13 chapter folders created (ch01-ch13)
- [ ] Each chapter has `index.md` with frontmatter + 4 sections (Objectives, Theory, Lab, Assessment)
- [ ] Chapter 1 fully written with 5 MCQs

### Phase 3: React Components ✅
- [ ] `src/components/MCQ/index.tsx` renders question, 4 options, submit button
- [ ] MCQ shows green/red feedback after submission
- [ ] `MCQ.test.tsx` passes with >80% coverage
- [ ] `src/components/CollapsibleLab/index.tsx` toggles visibility
- [ ] `src/pages/index.tsx` custom landing page with animations

### Phase 4: Hardware Page ✅
- [ ] `static/data/hardware-tiers.yaml` exists with 3 tiers (Economy, Proxy, Cloud)
- [ ] `docs/hardware-requirements.md` displays pricing tables
- [ ] All prices accurate (Jetson $249, RealSense $349, Unitree Go2 $3000)

### Phase 5: Static Assets ✅
- [ ] `src/css/custom.css` adds dark mode colors
- [ ] Mobile responsiveness tested (viewport <768px)
- [ ] README.md created with setup instructions

### Phase 6: Testing ✅
- [ ] `npm run test` passes (MCQ tests)
- [ ] `npm run build` succeeds with 0 errors
- [ ] Lighthouse CI scores >90 (Performance, Accessibility)
- [ ] No broken links (linkinator scan passes)

### Phase 7: Deployment ✅
- [ ] Code pushed to GitHub (`1-physical-ai-textbook` branch)
- [ ] GitHub Pages enabled (Settings > Pages > Source: GitHub Actions)
- [ ] GitHub Actions workflow passes (green checkmark)
- [ ] Site live at https://<username>.github.io/physical-ai-textbook/
- [ ] All features functional (navigation, MCQs, dark mode, mobile)

---

## /sp.check Validation Command

After completing all tasks, run the following validation script to ensure everything is covered:

```bash
#!/bin/bash
# Validation script: /sp.check

echo "🔍 Running Physical AI Textbook Validation Checks..."

# Check 1: Docusaurus project exists
if [ ! -d "docusaurus" ]; then
  echo "❌ FAIL: docusaurus/ directory not found"
  exit 1
fi
echo "✅ PASS: Docusaurus project exists"

# Check 2: 13 chapters exist
CHAPTER_COUNT=$(find docusaurus/docs -name "ch*" -type d | wc -l)
if [ "$CHAPTER_COUNT" -lt 13 ]; then
  echo "❌ FAIL: Only $CHAPTER_COUNT chapters found (expected 13)"
  exit 1
fi
echo "✅ PASS: 13 chapters exist"

# Check 3: MCQ component exists
if [ ! -f "docusaurus/src/components/MCQ/index.tsx" ]; then
  echo "❌ FAIL: MCQ component not found"
  exit 1
fi
echo "✅ PASS: MCQ component exists"

# Check 4: Hardware page exists
if [ ! -f "docusaurus/docs/hardware-requirements.md" ]; then
  echo "❌ FAIL: Hardware requirements page not found"
  exit 1
fi
echo "✅ PASS: Hardware requirements page exists"

# Check 5: GitHub Actions workflow exists
if [ ! -f ".github/workflows/deploy.yml" ]; then
  echo "❌ FAIL: GitHub Actions workflow not found"
  exit 1
fi
echo "✅ PASS: GitHub Actions workflow exists"

# Check 6: Build succeeds
cd docusaurus
npm run build > /dev/null 2>&1
if [ $? -ne 0 ]; then
  echo "❌ FAIL: Build failed"
  exit 1
fi
echo "✅ PASS: Build succeeds"

# Check 7: i18n stub exists
if [ ! -d "i18n/ur" ]; then
  echo "❌ FAIL: i18n/ur directory not found"
  exit 1
fi
echo "✅ PASS: i18n stub exists"

echo ""
echo "🎉 All validation checks passed!"
echo "✅ Docusaurus 3.x site scaffold complete"
echo "✅ 13 chapters with MDX templates"
echo "✅ MCQ React component with instant feedback"
echo "✅ Hardware Requirements page with 3 tiers"
echo "✅ GitHub Actions CI/CD pipeline"
echo "✅ i18n stub for Urdu translation"
echo "✅ Dark mode + mobile responsive"
echo ""
echo "Next steps:"
echo "1. Write chapter content (Theory, Labs, MCQs)"
echo "2. Add Mermaid diagrams to chapters"
echo "3. Deploy to GitHub Pages (push to main)"
echo "4. Consider running /sp.adr for architecture decisions"
```

**Save as**: `.specify/scripts/bash/validate-textbook.sh`

**Run**:
```bash
chmod +x .specify/scripts/bash/validate-textbook.sh
./.specify/scripts/bash/validate-textbook.sh
```

**Expected Output**: All checks pass with ✅

---

## Summary

**Total Tasks**: 32 main tasks + 1 validation script
**Estimated Time**: 12-25 hours (depending on parallelization)
**Priority Breakdown**:
- **[P1]**: 13 tasks (critical path: setup, infrastructure, deployment)
- **[P2]**: 11 tasks (core features: components, testing, content structure)
- **[P3]**: 8 tasks (enhancements: hardware page, styling, landing page)

**Parallelizable**: 12 tasks marked [P] can run concurrently

**Ready for Implementation**: All tasks are copy-paste-ready with exact commands. Follow phases sequentially, run parallel tasks concurrently where marked.

**Constitution Alignment**: All tasks validated against 7 principles (Helpful, Honest, Harmless, Spec-Driven, Structured, Efficient, Innovative).

**Next Command**: `/sp.implement` to execute tasks systematically, or start with T001 manually.
