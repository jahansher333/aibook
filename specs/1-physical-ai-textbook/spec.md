# Feature Specification: Physical AI & Humanoid Robotics Textbook

**Feature Branch**: `1-physical-ai-textbook`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description: "Physical AI & Humanoid Robotics Textbook - Target audience: College and University students and educators learning embodied intelligence. Focus: 13-chapter interactive textbook teaching ROS 2, Gazebo/Unity, NVIDIA Isaac Sim, and Vision-Language-Action (VLA) models for humanoid robots."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Access Interactive Learning Platform (Priority: P1) 🎯 MVP

College students and educators access a modern, interactive textbook platform to learn Physical AI and humanoid robotics through a visually engaging landing page with animations that introduce key concepts.

**Why this priority**: This is the foundation - users must be able to access and navigate the platform before engaging with any content. Landing page sets expectations and motivation for learners.

**Independent Test**: Navigate to deployed site URL, verify landing page loads with animations within 3 seconds on desktop/mobile, click "Start Learning" button to access chapter index.

**Acceptance Scenarios**:

1. **Given** a student visits the deployed site URL, **When** the page loads, **Then** they see an animated landing page showcasing Physical AI concepts (robot simulations, ROS 2 visualization, humanoid kinematics) with clear call-to-action "Start Learning"
2. **Given** the landing page is loaded, **When** the student clicks "Start Learning", **Then** they are navigated to the 13-chapter table of contents with progress tracking
3. **Given** a mobile user accesses the site, **When** they interact with the landing page, **Then** animations render smoothly (30+ FPS) and responsive design adapts to screen size

---

### User Story 2 - Navigate Chapter Structure (Priority: P1) 🎯 MVP

Students navigate through 13 chapters mapped to weekly course modules (Weeks 1-13 + Capstone), understanding the learning path from ROS 2 fundamentals to advanced humanoid control.

**Why this priority**: Clear navigation is essential for guided learning. Students need to understand the progression from basics (ROS 2) to advanced topics (VLA models).

**Independent Test**: From chapter index, click on any chapter (e.g., "Chapter 4: Gazebo Simulation"), verify it loads with table of contents showing Objectives, Theory, Hands-on Lab, and Assessment sections.

**Acceptance Scenarios**:

1. **Given** the student is on the chapter index, **When** they view the list, **Then** they see 13 chapters with descriptive titles matching Weeks 1-13 + Capstone
2. **Given** a student clicks on "Chapter 2: ROS 2 Fundamentals", **When** the chapter loads, **Then** they see four sections: Objectives, Theory, Hands-on Lab (with code snippets), and Assessment (MCQs)
3. **Given** a student completes Chapter 2's assessment, **When** they return to the index, **Then** their progress is visually indicated (e.g., checkmark, completion percentage)

---

### User Story 3 - Learn Through Structured Content (Priority: P2)

Students engage with each chapter's four-part structure (Objectives → Theory → Hands-on Lab → Assessment) to build understanding from learning goals through practical application to validation.

**Why this priority**: After accessing the platform (P1), students need high-quality educational content that follows pedagogical best practices (constructive alignment: objectives → content → assessment).

**Independent Test**: Open "Chapter 7: Vision-Language-Action Models", verify all four sections are present: 1) Objectives list learning outcomes, 2) Theory explains VLA concepts with diagrams, 3) Hands-on Lab provides executable code for Whisper + GPT planning, 4) Assessment includes 5-10 MCQs testing VLA understanding.

**Acceptance Scenarios**:

1. **Given** a student opens any chapter, **When** they scroll to the Objectives section, **Then** they see 3-5 measurable learning outcomes (e.g., "By the end of this chapter, you will be able to implement ROS 2 publisher-subscriber nodes")
2. **Given** a student reads the Theory section, **When** they encounter technical concepts (e.g., URDF, tf2), **Then** they find APA-cited references to official documentation (ROS 2 docs, NVIDIA Isaac guides) and inline diagrams (Markdown/SVG)
3. **Given** a student reaches the Hands-on Lab section, **When** they copy code snippets, **Then** all code is executable in the specified simulation environment (Isaac Sim or Gazebo) with clear setup instructions
4. **Given** a student completes a chapter's lab, **When** they proceed to the Assessment section, **Then** they answer 5-10 multiple-choice questions that test understanding of objectives, receive instant feedback (correct/incorrect), and see their score

---

### User Story 4 - Execute Hands-on Labs in Simulation (Priority: P2)

Students run all lab code in simulation environments (NVIDIA Isaac Sim or Gazebo Harmonic) before deploying to physical Jetson hardware, ensuring safe, accessible learning without upfront hardware costs.

**Why this priority**: Simulation-first approach removes hardware barriers and enables iterative learning. After understanding content structure (P3), students need practical coding experience.

**Independent Test**: In "Chapter 12: Hardware Integration", follow lab instructions to run a ROS 2 navigation script in Isaac Sim, verify simulated robot navigates to waypoint, then see deployment instructions for Jetson Orin Nano with exact steps.

**Acceptance Scenarios**:

1. **Given** a student is in a Hands-on Lab section, **When** they follow setup instructions, **Then** they can execute code in Isaac Sim (for VSLAM/Nav2) or Gazebo (for ROS 2 basics) on their local machine or cloud VM
2. **Given** a student completes a simulation lab, **When** they review deployment notes, **Then** they see clear instructions for transferring code to Jetson Orin Nano hardware with environment differences documented
3. **Given** a student without Isaac Sim license, **When** they access labs requiring it, **Then** they see alternative Gazebo implementations or cloud VM options (e.g., AWS EC2 g4dn instances with Isaac Sim pre-installed)

---

### User Story 5 - Understand Hardware Requirements (Priority: P3)

Students and educators review three budget tiers (Economy Jetson Kit ~$700, Unitree Go2 proxy ~$3k, Cloud OpEx ~$205/quarter) to make informed decisions about hardware investment vs. cloud rental.

**Why this priority**: Hardware planning is crucial but secondary to learning core concepts (P1-P2). Students need this early to plan budgets but can start learning in simulation immediately.

**Independent Test**: Navigate to "Hardware Requirements" section (linked from landing page and Chapter 1), verify three tiers are listed with exact component breakdowns (Jetson Orin Nano $249 + RealSense D435i $349 = ~$700 for Economy tier), see cloud cost calculations for quarterly rental.

**Acceptance Scenarios**:

1. **Given** a student accesses the Hardware Requirements section, **When** they view the Economy tier, **Then** they see itemized costs: Jetson Orin Nano Developer Kit ($249), Intel RealSense D435i ($349), power supply ($50), microSD card ($30), total ~$700
2. **Given** an educator planning a lab, **When** they review the Unitree Go2 tier, **Then** they see robot specifications (proxy for G1 humanoid, $3000 base price), use cases (whole-body control, locomotion testing), and ROI analysis (reusable across multiple semesters)
3. **Given** a student preferring cloud-only workflow, **When** they review Cloud OpEx tier, **Then** they see quarterly cost breakdown (~$205/quarter for AWS EC2 g4dn.xlarge with Isaac Sim, billed hourly for 40 hours/quarter lab time), comparison to hardware CapEx, and setup guides

---

### User Story 6 - Complete Capstone Project (Priority: P3)

Advanced students complete a capstone project (Chapter 13) synthesizing all skills: building a voice-command autonomous humanoid that uses Whisper (speech-to-text), GPT-4 (planning), ROS 2 (control), and Isaac Sim (simulation) → deployable to Jetson.

**Why this priority**: Capstone validates mastery of all 12 prior chapters. It's the culmination but not required for foundational learning (P1-P2).

**Independent Test**: Follow Chapter 13 instructions to implement voice-command system: speak "Pick up the red cube" → Whisper transcribes → GPT-4 generates manipulation plan → ROS 2 executes in Isaac Sim → see humanoid arm grasp cube. Verify code runs in simulation and includes Jetson deployment checklist.

**Acceptance Scenarios**:

1. **Given** a student has completed Chapters 1-12, **When** they start Chapter 13, **Then** they see a project specification: "Build a voice-command humanoid that responds to natural language instructions for manipulation and navigation tasks"
2. **Given** a student follows capstone lab instructions, **When** they integrate Whisper, GPT-4, and ROS 2 nodes, **Then** the system accepts voice input, generates a task plan, and executes it in Isaac Sim with visual feedback
3. **Given** a student completes the capstone simulation, **When** they review assessment criteria, **Then** they see rubric covering: voice accuracy (Whisper), plan validity (GPT-4), execution success (ROS 2), and code quality (documented, modular)

---

### Edge Cases

- **What happens when a student's machine cannot run Isaac Sim locally?**
  Provide cloud VM setup guide (AWS EC2 g4dn.xlarge with NVIDIA drivers, Isaac Sim installed) and cost estimator (~$1.20/hour, ~$205/quarter for 40 hours). Include Gazebo fallback for labs not requiring Isaac-specific features.

- **How does the system handle outdated software versions (e.g., ROS 2 Humble → Jazzy)?**
  Each chapter specifies software versions in prerequisites (e.g., "ROS 2 Humble on Ubuntu 22.04"). Include migration notes for major version changes and lock dependencies in code examples (pip requirements.txt, apt package versions).

- **What if MCQ answers are unclear or students dispute correctness?**
  Each MCQ includes explanations for correct/incorrect answers with citations to Theory section or external docs. Provide feedback form link in Assessment sections for reporting issues.

- **How does the site handle users without JavaScript (for animations)?**
  Landing page animations degrade gracefully to static images. Core content (chapters, MCQs) functions without JS using semantic HTML. Docusaurus ensures progressive enhancement.

- **What happens when hardware prices change significantly (e.g., Jetson Orin Nano discontinued)?**
  Hardware Requirements section includes "Last Updated: [DATE]" and links to live pricing sources (NVIDIA store, Amazon). Include note: "Prices subject to change; verify with vendors before purchase."

## Requirements *(mandatory)*

### Functional Requirements

#### Site Generation & Deployment

- **FR-001**: System MUST generate a Docusaurus 3.x site with Next.js integration for React Server Components
- **FR-002**: System MUST deploy to GitHub Pages OR Vercel with automated CI/CD (GitHub Actions workflow)
- **FR-003**: Site MUST be fully static (no server-side rendering required) with client-side interactivity for MCQs and animations

#### Landing Page

- **FR-004**: Landing page MUST include animated visuals showcasing Physical AI concepts (robot arm manipulation, bipedal locomotion, sensor visualization)
- **FR-005**: Animations MUST use CSS animations or React Spring (no proprietary video files) to ensure fast load times (<3s)
- **FR-006**: Landing page MUST include clear navigation to chapter index with call-to-action button ("Start Learning")

#### Chapter Structure

- **FR-007**: System MUST provide exactly 13 chapters mapped to Weeks 1-13 + Capstone:
  1. Introduction to Physical AI and Embodied Intelligence
  2. ROS 2 Fundamentals (nodes, topics, services, actions)
  3. Robot Modeling (URDF, TF2, Xacro, visualization)
  4. Gazebo Simulation (physics engines, sensor plugins, world files)
  5. Unity Simulation (ML-Agents integration, high-fidelity rendering)
  6. NVIDIA Isaac Sim (VSLAM, Nav2 path planning, GPU acceleration)
  7. Vision-Language-Action Models (Whisper speech-to-text, GPT-4 planning, CLIP vision)
  8. Humanoid Kinematics (forward/inverse kinematics, Jacobian, DH parameters)
  9. Locomotion (bipedal walking, ZMP control, trajectory optimization)
  10. Manipulation (grasping, force control, motion planning with MoveIt2)
  11. Conversational AI Integration (GPT-4 for natural language robot commands)
  12. Hardware Integration (Jetson Orin Nano setup, RealSense D435i, sensor fusion)
  13. Capstone Project (voice-command autonomous humanoid with full pipeline)

- **FR-008**: Each chapter MUST contain four sections in order: Objectives, Theory, Hands-on Lab, Assessment
- **FR-009**: Objectives section MUST list 3-5 measurable learning outcomes using Bloom's taxonomy verbs (e.g., "implement", "analyze", "design")
- **FR-010**: Theory section MUST include technical explanations with inline diagrams (Markdown/SVG) and APA citations to official docs (ROS 2, NVIDIA, OpenAI)
- **FR-011**: Hands-on Lab section MUST provide executable code snippets in Markdown fenced code blocks with language tags (```python, ```bash, ```xml)
- **FR-012**: Assessment section MUST include 5-10 multiple-choice questions per chapter with instant feedback (correct/incorrect) and explanations

#### MCQ Functionality

- **FR-013**: MCQ system MUST be implemented in React with state management (useState) for answer selection
- **FR-014**: MCQs MUST show immediate feedback when student selects an answer (highlight correct in green, incorrect in red)
- **FR-015**: Each MCQ MUST include an explanation referencing the Theory section or cited source
- **FR-016**: MCQ scores MUST be calculated per chapter (percentage correct out of total questions)

#### Hardware Requirements Section

- **FR-017**: Site MUST include a dedicated Hardware Requirements page (linked from landing page and Chapter 1)
- **FR-018**: Hardware section MUST detail three tiers with exact component lists and prices:
  - **Economy Tier**: Jetson Orin Nano Developer Kit ($249), Intel RealSense D435i ($349), accessories ($~100), total ~$700
  - **Proxy Tier**: Unitree Go2 quadruped ($3000, serves as testbed for humanoid algorithms), Jetson Xavier ($~500), RealSense ($349), total ~$3850
  - **Cloud OpEx Tier**: AWS EC2 g4dn.xlarge (~$1.20/hour, ~$205/quarter for 40 hours lab time), Isaac Sim pre-installed AMI, storage costs (~$50/quarter)
- **FR-019**: Each hardware tier MUST include use case recommendations (e.g., Economy for solo learners, Proxy for research labs, Cloud for institutions without CapEx budgets)

#### Labs & Simulation

- **FR-020**: All labs MUST be executable in simulation first (Isaac Sim for VSLAM/Nav2/advanced features, Gazebo for ROS 2 basics)
- **FR-021**: Labs MUST include setup instructions specifying software versions (ROS 2 Humble, Ubuntu 22.04, Isaac Sim 2023.1.0)
- **FR-022**: Labs requiring Jetson deployment MUST include deployment guides with environment differences (ARM64 vs x86_64, GPU drivers, sensor permissions)
- **FR-023**: Code examples MUST be self-contained (include dependencies in comments or requirements.txt) and reproducible

#### Content Constraints

- **FR-024**: All content MUST be in Markdown (.md files) for version control and accessibility
- **FR-025**: Citations MUST use APA format with inline links (e.g., "[ROS 2 Humble documentation](https://docs.ros.org/en/humble/)")
- **FR-026**: Site MUST NOT include vendor marketing content or paid API integrations (use free tiers: OpenAI free trial, Groq free tier for demos)
- **FR-027**: Code examples MUST use open-source libraries (ROS 2, PyTorch, Transformers, Isaac Sim free personal license)

### Key Entities *(include if feature involves data)*

- **Chapter**: Represents one of 13 course modules
  - Attributes: chapter number (1-13), title, week mapping, four sections (Objectives, Theory, Lab, Assessment), completion status
  - Relationships: Contains multiple MCQs, belongs to course structure

- **MCQ (Multiple-Choice Question)**: Assessment item within a chapter
  - Attributes: question text, 4 answer options, correct answer index, explanation text, difficulty level
  - Relationships: Belongs to Chapter.Assessment section

- **Hardware Tier**: Budget configuration for physical setup
  - Attributes: tier name (Economy/Proxy/Cloud), component list with prices, total cost, use case recommendations
  - Relationships: Referenced by Chapter 1 and Hardware Requirements page

- **Lab Code Snippet**: Executable code example within Hands-on Lab
  - Attributes: language (Python/Bash/XML), code content, prerequisites, expected output, simulation environment (Isaac Sim/Gazebo)
  - Relationships: Belongs to Chapter.Lab section, may reference Hardware Tier for deployment

- **Learning Objective**: Measurable outcome for a chapter
  - Attributes: Bloom's taxonomy verb, skill description, assessment method
  - Relationships: Belongs to Chapter.Objectives section, validated by Chapter.Assessment MCQs

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can navigate from landing page to any of 13 chapters within 10 seconds (max 3 clicks)
- **SC-002**: Landing page animations load and play smoothly on desktop (Chrome/Firefox/Safari) and mobile (iOS Safari/Chrome) within 3 seconds on 10 Mbps connection
- **SC-003**: All 13 chapters are accessible with complete four-section structure (Objectives, Theory, Lab, Assessment), verified by manual review or automated script counting sections per chapter
- **SC-004**: Students can copy code snippets from Hands-on Labs, paste into Isaac Sim or Gazebo, and execute without errors in specified environment (tested on 5 representative labs: ROS 2 publisher, URDF loading, Gazebo world, Isaac Nav2, VLA integration)
- **SC-005**: MCQ functionality allows students to select answers, see instant feedback (correct/incorrect highlighting), and view explanations for all 65-130 total questions (5-10 per chapter × 13 chapters)
- **SC-006**: Hardware Requirements page lists exact component prices with sources (links to NVIDIA store, Amazon, AWS calculator) and "Last Updated" date within 30 days of deployment
- **SC-007**: Site passes accessibility audit (WCAG 2.1 AA) for color contrast, keyboard navigation, and screen reader compatibility using Lighthouse audit (score 90+)
- **SC-008**: Deployed site loads globally with <5 second time-to-interactive (TTI) measured by Lighthouse on simulated 4G connection
- **SC-009**: All APA citations link to live URLs (ROS 2 docs, NVIDIA guides, OpenAI API docs) with <5% broken link rate verified by automated link checker
- **SC-010**: Capstone project (Chapter 13) lab code successfully integrates Whisper + GPT-4 + ROS 2 in Isaac Sim, demonstrated by test run completing voice-command task (e.g., "Pick up the red cube" → robot executes grasp)

### Non-Functional Success Criteria

- **SC-011**: Site repository passes `/sp.analyze` validation with zero gaps (no missing sections in spec.md, plan.md, tasks.md)
- **SC-012**: Site build completes in <5 minutes on GitHub Actions or Vercel CI/CD pipeline
- **SC-013**: Static site bundle size is <10 MB (excluding external video/images), ensuring fast CDN delivery
- **SC-014**: 95% of users (surveyed post-course) report ability to execute at least one hands-on lab successfully in simulation
- **SC-015**: Content quality reviewed by 2+ subject matter experts (robotics educators or industry practitioners) with 0 critical feedback items (factual errors, unsafe practices) before v1.0 release

## Assumptions

1. **Target Audience Technical Prerequisites**: Students have basic Python programming knowledge (loops, functions, classes) and Linux CLI familiarity (cd, ls, apt, git). No prior ROS or robotics experience assumed.

2. **Software Environment**: Students have access to one of:
   - Local Linux machine (Ubuntu 22.04, 16GB RAM, NVIDIA GPU with 6GB+ VRAM for Isaac Sim)
   - Cloud VM (AWS EC2 g4dn.xlarge or equivalent, provided by institution or self-funded)
   - Docker containers with ROS 2 Humble and Gazebo (for chapters not requiring Isaac Sim)

3. **Internet Connectivity**: Students have reliable internet (10 Mbps+) for downloading software dependencies (ROS 2 packages ~2GB, Isaac Sim ~20GB) and accessing cloud VMs.

4. **Hardware Acquisition Timeline**: Students and educators have 2-4 weeks lead time to order Jetson kits or arrange cloud budgets before hands-on labs begin (Chapter 1-2 are theory-heavy, allowing buffer).

5. **Open-Source Licensing**: All code examples use permissive licenses (MIT, Apache 2.0, BSD) compatible with educational and commercial use. Isaac Sim Personal License allows free educational use.

6. **API Rate Limits**: Free tier APIs (OpenAI, Groq) are used for demonstrations only. Production deployments (e.g., capstone voice commands) require students to use personal API keys with usage under free tier limits (~$5/month for testing).

7. **Content Maintenance**: Hardware prices and software versions are reviewed quarterly. Major version updates (e.g., ROS 2 Jazzy) trigger content revisions with migration guides.

8. **Accessibility Standards**: Site meets WCAG 2.1 AA standards, prioritizing keyboard navigation and screen reader compatibility. Animations include prefers-reduced-motion CSS media queries for users with motion sensitivity.

9. **Deployment Platform**: GitHub Pages is default deployment target (free for public repos), with Vercel as alternative (free tier supports unlimited bandwidth for hobby projects).

10. **Simulation Performance**: Isaac Sim labs may run at 10-20 FPS on minimum-spec machines (RTX 2060, 16GB RAM). Performance optimization guides included for users with lower-end hardware.

## Dependencies

### External Dependencies

1. **Docusaurus 3.x Framework**: Requires Node.js 18+ and npm/yarn. Documentation site generator with React integration.
   - Risk: Breaking changes in future Docusaurus versions.
   - Mitigation: Pin Docusaurus version in package.json, test updates in staging.

2. **ROS 2 Humble Hawksbill**: Ubuntu 22.04 LTS, binary packages via apt. EOL May 2027.
   - Risk: Ubuntu 22.04 or ROS 2 Humble reaches EOL during textbook lifespan.
   - Mitigation: Include migration guide to next LTS (ROS 2 Jazzy on Ubuntu 24.04) in Chapter 1 prerequisites.

3. **NVIDIA Isaac Sim 2023.1.0+**: Requires NVIDIA GPU (RTX series), Isaac Sim Omniverse launcher. Free Personal License.
   - Risk: License terms change, or Isaac Sim deprecates features used in labs.
   - Mitigation: Archive Isaac Sim 2023.1.0 Docker image for reproducibility. Provide Gazebo alternatives for critical labs.

4. **GitHub Pages / Vercel**: Free hosting for static sites. GitHub Pages has 1GB repo size limit, 100GB monthly bandwidth soft limit.
   - Risk: Bandwidth exceeded if site becomes highly trafficked.
   - Mitigation: Use CDN for large assets (videos, datasets). Vercel free tier has no bandwidth limit for hobby projects.

5. **OpenAI API (Free Tier)**: Whisper and GPT-4 demos in Chapter 7, 11, 13. Free tier: $5 credit, rate limits.
   - Risk: Free tier exhausted during demos, or API terms change.
   - Mitigation: Use Groq free tier (faster inference) as primary demo platform. Include Hugging Face local model alternatives (Whisper.cpp, GPT4All).

### Internal Dependencies

1. **Constitution Principles**: Content must align with "Honest and Accurate" (accurate hardware prices) and "Harmless and Inclusive" (safety warnings, sim-first approach).
   - Dependency: `.specify/memory/constitution.md` v1.0.0
   - Validation: Each chapter reviewed against constitution checklist.

2. **Spec-Kit Plus Templates**: Spec, Plan, Tasks must use official templates from `.specify/templates/`.
   - Dependency: `spec-template.md`, `plan-template.md`, `tasks-template.md`
   - Validation: Run `/sp.analyze` to check template compliance.

3. **Hardware Pricing Data**: Accurate as of 2025-12-09. Requires quarterly updates.
   - Dependency: Live pricing from NVIDIA store, Amazon, AWS calculator.
   - Update Process: Quarterly review (March, June, Sept, Dec) with price adjustments in Hardware Requirements page.

### Third-Party Services

1. **AWS EC2 (Cloud OpEx Tier)**: g4dn.xlarge instances (~$1.20/hour). Free tier: 750 hours/month for first 12 months (t2.micro only, not applicable here).
   - Risk: AWS pricing changes or instance types deprecated.
   - Mitigation: Document alternative cloud providers (Azure NC6 Promo, Google Cloud N1 with T4 GPU).

2. **GitHub Actions (CI/CD)**: 2000 minutes/month free for public repos. Build + deploy takes ~5 minutes/run.
   - Risk: Exceeded free tier (>400 builds/month).
   - Mitigation: Cache dependencies (Node modules, Docusaurus build) to reduce build time. Use Vercel as fallback (unlimited builds for hobby projects).

## Out of Scope

1. **Live Video Lectures**: No recorded video content. Focus is on written tutorials, diagrams, and code. Rationale: Keeps repo size small, accessible for low-bandwidth users, easier to maintain/update.

2. **Interactive 3D Simulations in Browser**: No WebGL-based robot simulators embedded in site. Labs run in native Isaac Sim/Gazebo. Rationale: Browser-based 3D requires heavy JavaScript, poor performance on mobile, complex maintenance.

3. **User Accounts / Progress Tracking Backend**: No server-side user authentication or database for tracking quiz scores across sessions. Rationale: Static site constraint (GitHub Pages/Vercel free tier). Progress stored in browser localStorage only.

4. **Real-Time Collaboration**: No shared lab environments or multiplayer simulations. Rationale: Requires server infrastructure beyond static site scope.

5. **Hardware Vendor Partnerships**: No affiliate links, sponsored content, or exclusive deals with Jetson/Unitree vendors. Rationale: Maintains educational neutrality per constitution's "Honest and Accurate" principle.

6. **Non-English Translations**: Urdu translation is out of scope for this initial spec (handled separately as bonus feature in constitution). Rationale: Focuses on English-language MVP. Translation as iterative enhancement.

7. **Advanced Research Topics**: No coverage of cutting-edge topics like reinforcement learning for locomotion (PPO, SAC) or transformer-based policies (Diffusion Policy). Rationale: Textbook targets foundational skills. Advanced topics in future "Part 2" textbook.

8. **Mobile App (iOS/Android)**: No native mobile applications. Site is mobile-responsive web only. Rationale: Static site deployed via browser, no app store complexity.

9. **Automated Grading System**: MCQ feedback is instant but scores not submitted to instructor dashboard. Rationale: Static site, no backend. Educators can review answers manually or use LMS integration (future enhancement).

10. **Physical Robot Kits Included**: No bundled hardware sales or pre-configured robot kits. Rationale: Educational content only. Students source hardware independently per budget tier guidelines.

## Risks & Mitigations

### Technical Risks

1. **Risk**: Isaac Sim requires high-end GPU (RTX 2060+), excluding students with low-spec machines.
   - **Impact**: High (blocks ~30% of target audience per typical student hardware surveys)
   - **Mitigation**: Provide Gazebo fallback for 70% of labs. Include cloud VM setup guide (AWS EC2 g4dn.xlarge) with cost estimator. Offer Docker image with Isaac Sim headless mode for remote rendering.

2. **Risk**: Docusaurus build fails due to Markdown syntax errors or plugin incompatibilities.
   - **Impact**: Medium (blocks deployment, but fixable)
   - **Mitigation**: CI/CD pipeline runs `docusaurus build` on every commit. Lint Markdown files with `markdownlint` pre-commit hook. Pin all Docusaurus plugins to specific versions in package.json.

3. **Risk**: MCQ React components break on older browsers (IE11, Safari <14).
   - **Impact**: Low (target audience uses modern browsers per institution requirements)
   - **Mitigation**: Test on Chrome, Firefox, Safari latest versions. Use Babel transpilation for ES6+ features. Provide graceful degradation (MCQs render as static text if JS disabled).

### Content Risks

4. **Risk**: Hardware prices fluctuate significantly (e.g., Jetson Orin Nano out of stock, price increases 50%).
   - **Impact**: High (misleads students on budget planning)
   - **Mitigation**: Include "Last Updated" date on Hardware Requirements page. Quarterly price review. Link to live vendor pages (NVIDIA store, Amazon) for real-time prices. Add disclaimer: "Prices subject to change; verify before purchase."

5. **Risk**: ROS 2 Humble or Isaac Sim major version updates break lab code.
   - **Impact**: High (all labs become non-functional)
   - **Mitigation**: Lock software versions in prerequisites (ROS 2 Humble, Ubuntu 22.04, Isaac Sim 2023.1.0). Archive Docker images with exact versions. Provide migration guides when updates needed.

6. **Risk**: Cited documentation URLs break (ROS 2 docs reorganize, NVIDIA moves Isaac guides).
   - **Impact**: Medium (frustrates students, breaks learning flow)
   - **Mitigation**: Run automated link checker (e.g., `linkinator`) monthly via cron job. Archive critical docs in repo's `docs/references/` folder as PDF backups.

### Audience Risks

7. **Risk**: Students lack Python/Linux prerequisites, cannot follow labs.
   - **Impact**: Medium (slows learning, increases support requests)
   - **Mitigation**: Chapter 1 includes "Prerequisites Check" section with self-assessment quiz (Python loops, Linux commands). Link to free prerequisite courses (e.g., Python Crash Course, Linux Survival Guide). Provide supplementary "Appendix A: Python Refresher" and "Appendix B: Linux Cheat Sheet."

8. **Risk**: Educators adopt textbook but institution lacks budget for hardware or cloud.
   - **Impact**: High (limits practical learning)
   - **Mitigation**: Emphasize Economy tier (~$700) as DIY option. Provide grant writing template for educators to request funding (NSF, institutional teaching innovation grants). Partner with cloud providers for educational credits (AWS Educate, Google Cloud Education).

### Deployment Risks

9. **Risk**: GitHub Pages or Vercel experiences outage during peak usage (start of semester).
   - **Impact**: Medium (students cannot access site)
   - **Mitigation**: Deploy to both GitHub Pages (primary) and Vercel (mirror) with DNS failover. Cache static assets on Cloudflare CDN for additional redundancy.

10. **Risk**: Repository exceeds GitHub's 1GB size limit due to large images or datasets.
    - **Impact**: Low (preventable with monitoring)
    - **Mitigation**: Use Git LFS for images >1MB. Host large datasets (robot models, sensor logs) on external storage (Hugging Face Datasets, Zenodo) with download links in labs. Monitor repo size with `git count-objects -vH` in CI/CD.

## Notes

- **Pedagogical Approach**: Follows constructive alignment (Objectives → Theory → Lab → Assessment) and Bloom's taxonomy (knowledge → application → analysis). Each chapter scaffolds from foundational ROS 2 (Chapters 1-3) to advanced VLA models (Chapter 7) and integration (Chapters 11-13).

- **Open Educational Resources (OER)**: This textbook is designed as an OER under Creative Commons BY-SA 4.0 license. Educators can remix, adapt, and redistribute with attribution. Code examples use MIT License.

- **Equity & Inclusion**: Sim-first approach ensures students without $700+ hardware budgets can participate fully. Cloud OpEx tier (~$205/quarter) provides pay-as-you-go alternative to upfront CapEx. No paid APIs in core content (free tiers only).

- **Industry Alignment**: Skills taught (ROS 2, Isaac Sim, VLA models) align with 2024-2025 robotics industry job postings (humanoid robot startups, autonomous vehicle companies, warehouse automation). Chapter 13 capstone project mimics real-world task: voice-command robot for human-robot interaction.

- **Future Enhancements** (Post-MVP):
  - **Better-Auth Integration**: Signup/signin with personalization quiz (hardware/software background) to tailor content. [Constitution bonus feature]
  - **Urdu Translation**: Per-chapter translation buttons using LiteLLM/Groq. [Constitution bonus feature]
  - **RAG Chatbot**: Embedded Q&A bot using OpenAI Agents SDK + Qdrant for context-aware help. [Constitution bonus feature]
  - **LMS Integration**: SCORM export for Canvas/Moodle, allowing instructors to import chapters into existing courses.
  - **Certification Path**: End-of-course badge/certificate for students completing all 13 assessments (80%+ score threshold).

- **Maintenance Plan**:
  - Quarterly reviews (March, June, Sept, Dec) for hardware pricing, software versions, broken links.
  - Annual major update aligned with ROS 2 LTS releases (May even years: 2026, 2028).
  - Community contributions via GitHub Issues/PRs for typo fixes, lab improvements, alternative solutions.

---

**Specification Complete**: Ready for `/sp.clarify` (if clarifications needed) or `/sp.plan` (architectural planning phase).
