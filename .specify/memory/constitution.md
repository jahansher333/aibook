<!--
SYNC IMPACT REPORT (Version Update)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Version Change: [INITIAL_VERSION] → 1.0.0
Change Type: MAJOR (Initial ratification of comprehensive constitution)
Date: 2025-12-09

Added Sections:
  - All 7 core principles (Helpful and Impactful, Honest and Accurate, Harmless and Inclusive,
    Spec-Driven and AI-Native, Structured and Comprehensive, Efficient and Scalable,
    Innovative yet Practical)
  - Technology Stack section
  - Development Workflow section
  - Governance section with versioning policy

Modified Principles: N/A (Initial version)
Removed Sections: N/A (Initial version)

Templates Requiring Updates:
  ✅ plan-template.md - Verified: Constitution Check section aligns with principles
  ✅ spec-template.md - Verified: User story structure supports educational/personalization goals
  ✅ tasks-template.md - Verified: Task structure supports modular agent-driven development

Follow-up TODOs: None (all placeholders filled)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
-->

# Physical AI & Humanoid Robotics Hackathon Project Constitution

## Core Principles

### I. Helpful and Impactful

**PRINCIPLE**: Deliver hands-on, accessible content that bridges digital AI to physical robotics, maximizing educational value through personalization.

**RULES**:
- MUST structure content from beginner (basic Python/Linux knowledge) to advanced (sim-to-real transfers)
- MUST include executable code, diagrams (Markdown/SVG), and labs tied to course modules
- MUST provide personalization based on user backgrounds via signup quizzes (hardware/software experience)
- MUST include interactive assessments tied to RAG chatbot for adaptive learning
- MUST deliver 13 chapters mirroring weekly course breakdown with clear learning objectives per module

**RATIONALE**: Physical AI requires bridging theoretical concepts with practical implementation. Personalization ensures inclusivity for users with varying hardware access (RTX workstations vs. cloud-only) and experience levels.

---

### II. Honest and Accurate

**PRINCIPLE**: Ground all content in verified sources and provided specifications; never invent specs, costs, or tools.

**RULES**:
- MUST cite all sources (ROS 2 docs, NVIDIA Isaac guides, hardware vendors)
- MUST detail accurate hardware tiers with exact components and prices:
  - Digital Twin Workstation: RTX 4070+ ($2000+)
  - Edge Kit: Jetson Orin Nano ($249) + RealSense D435i ($349) = ~$700
  - Robot Tiers: Unitree Go2 ($3000 proxy) to G1 ($16000 premium)
- MUST warn on risks: "Never control real robots from cloud due to latency—use local Jetson flashing"
- MUST NOT hallucinate API endpoints, data contracts, or cloud service costs
- MUST mark unclear requirements with "NEEDS CLARIFICATION" flags in specs

**RATIONALE**: Educational content integrity depends on accuracy. Misinformation about costs ($700 Edge Kit is significant investment) or safety (cloud latency causing robot accidents) can cause real harm.

---

### III. Harmless and Inclusive

**PRINCIPLE**: Promote safe practices and ensure inclusivity across linguistic, hardware, and experience barriers.

**RULES**:
- MUST include safety warnings in all robot control chapters (latency, collision avoidance)
- MUST support Urdu translation via per-chapter buttons using LiteLLM/Groq API (cached in Qdrant)
- MUST personalize content based on user hardware profiles (e.g., simplify GPU-intensive tasks for non-RTX users with cloud alternatives)
- MUST use open-source/free tiers: Neon Postgres (free tier), Qdrant Cloud (free tier), OpenAI Agents SDK (freemium)
- MUST provide alternative workflows for users without physical hardware (sim-only paths)
- MUST NOT create accessibility barriers through costs or locked platforms

**RATIONALE**: Robotics involves physical risk (moving parts, high voltages). Linguistic inclusivity (Urdu) expands access to South Asian learners. Hardware inclusivity ensures students without $2k+ workstations can participate via cloud alternatives.

---

### IV. Spec-Driven and AI-Native (Spec-Kit Plus Integration)

**PRINCIPLE**: Begin with Spec-Kit Plus specs (user stories, templates, patterns) and leverage Claude Code subagents/skills for AI-native development.

**RULES**:
- MUST generate user stories for all features (e.g., "As a student, I want ROS 2 node tutorials so I can control simulated humanoids")
- MUST use Spec-Kit Plus templates for specs (`spec.md`), plans (`plan.md`), and tasks (`tasks.md`)
- MUST leverage Claude Code subagents for modular tasks:
  - `BookWriterAgent`: Generate chapter content based on course modules
  - `SpecValidatorAgent`: Validate specs against constitution principles
  - `RAGEmbeddingAgent`: Process book chunks for vector DB
  - `TranslationAgent`: Manage Urdu translations with caching
- MUST create reusable skills:
  - `HardwarePersonalizerSkill`: Query user profile from Neon, return tailored content variants
  - `TranslateToUrduSkill`: Translate text using LiteLLM/Groq, cache in Qdrant
- MUST output YAML/JSON specs for book structure, RAG pipeline, and agent configs
- MUST generate Architecture Decision Records (ADRs) for significant decisions (e.g., "Why Docusaurus over GitBook", "Why Qdrant over Pinecone")

**RATIONALE**: AI-native development (agents generating agents) maximizes efficiency. Spec-driven approach ensures traceability (user story → task → implementation). Reusable skills prevent duplication across chapters.

---

### V. Structured and Comprehensive

**PRINCIPLE**: Output a complete, deployable project structure with all deliverables specified.

**RULES**:
- MUST provide full repo structure including:
  ```
  physical-ai-textbook/
  ├── docusaurus/                 # Book site (GitHub Pages deployment)
  │   ├── docs/                   # 13 chapters (ch01-intro, ch02-ros2, ..., ch13-capstone)
  │   ├── src/components/         # React components (RAG chatbot UI, PersonalizeButton, TranslateButton)
  │   └── docusaurus.config.js    # Docusaurus config with custom plugins
  ├── backend/                    # FastAPI RAG service
  │   ├── app/
  │   │   ├── main.py             # FastAPI app with OpenAI Agents SDK
  │   │   ├── models.py           # Neon Postgres models (User, Chapter, Translation)
  │   │   ├── auth.py             # Better-Auth integration
  │   │   ├── rag.py              # RAG pipeline (Qdrant retrieval + OpenAI generation)
  │   │   └── agents/             # Agent configs (YAML)
  │   └── requirements.txt        # FastAPI, OpenAI SDK, Qdrant, Neon, Better-Auth
  ├── specs/                      # Spec-Kit Plus artifacts
  │   ├── physical-ai-textbook/
  │   │   ├── spec.md             # Feature spec with user stories
  │   │   ├── plan.md             # Implementation plan
  │   │   └── tasks.md            # Task breakdown
  │   └── rag-chatbot/
  │       └── spec.md             # RAG chatbot spec
  ├── history/
  │   ├── prompts/                # Prompt History Records
  │   └── adr/                    # Architecture Decision Records
  ├── .github/workflows/
  │   └── deploy.yml              # GitHub Actions for Pages deployment
  └── README.md                   # Setup guide and verification checklist
  ```
- MUST include 13 chapters mirroring course modules:
  1. Introduction to Physical AI and Embodied Intelligence
  2. ROS 2 Fundamentals (nodes/topics/services/actions)
  3. Robot Modeling (URDF, TF2, visualization)
  4. Gazebo Simulation (physics engines, sensors)
  5. Unity Simulation (high-fidelity graphics, ML-Agents)
  6. NVIDIA Isaac Sim (VSLAM, Nav2 path planning)
  7. Vision-Language-Action Models (Whisper + GPT planning)
  8. Humanoid Kinematics (forward/inverse kinematics)
  9. Locomotion (bipedal walking, balance control)
  10. Manipulation (grasping, object interaction)
  11. Conversational AI Integration (GPT for robot commands)
  12. Hardware Integration (Jetson setup, RealSense)
  13. Capstone Project (voice-command autonomous humanoid)
- MUST embed RAG chatbot in all chapters via React component
- MUST include per-chapter "Personalize Content" and "Translate to Urdu" buttons

**RATIONALE**: Comprehensive structure ensures no missing pieces during hackathon submission. 13-chapter breakdown mirrors weekly course plan, ensuring complete coverage. Embedded features (RAG, personalization, translation) maximize bonus points.

---

### VI. Efficient and Scalable

**PRINCIPLE**: Keep code modular, leverage free tiers, and optimize for rapid deployment.

**RULES**:
- MUST use free tiers for all services:
  - Neon Postgres: Free tier (3 GB storage)
  - Qdrant Cloud: Free tier (1 GB vectors)
  - OpenAI API: Freemium (user provides key)
  - Groq/LiteLLM: Free tier (fast translation)
- MUST deploy within 1 hour using:
  - GitHub Actions for Docusaurus Pages deployment
  - Vercel for FastAPI backend (or Railway/Render free tier)
- MUST optimize for low latency:
  - Local JavaScript for UI interactions (buttons)
  - Qdrant caching for repeated translations/queries
  - OpenAI Agents SDK streaming for real-time chatbot responses
- MUST keep subagents modular: Each agent generates independent artifacts (chapters, configs)
- MUST avoid monolithic code: Separate concerns (auth, RAG, content generation)

**RATIONALE**: Hackathon time constraints require rapid deployment. Free tiers minimize costs for students. Modular architecture enables parallel development by subagents. Low latency ensures usable chatbot (no 5-second response times).

---

### VII. Innovative yet Practical

**PRINCIPLE**: Infuse AI-native elements (agents, dynamic content) while ensuring cross-browser compatibility and testability.

**RULES**:
- MUST use subagents for dynamic content generation:
  - `BookWriterAgent` generates chapter variants (beginner vs. advanced) based on user profile
  - `PersonalizationAgent` adapts labs (cloud alternatives for non-RTX users)
- MUST use skills for reusable logic:
  - `TranslateToUrduSkill` with caching to avoid redundant API calls
  - `HardwarePersonalizerSkill` to query Neon and return tailored content
- MUST test via pseudocode in plan phase (verify RAG retrieval before full implementation)
- MUST ensure cross-browser/mobile compatibility:
  - Docusaurus responsive themes
  - ChatKit UI mobile-tested
  - Translation buttons work on iOS/Android browsers
- MUST provide fallback mechanisms:
  - If Qdrant unavailable, RAG degrades to direct OpenAI calls (no retrieval)
  - If translation API fails, show English with error message

**RATIONALE**: AI-native approach (agents generating content) showcases innovation. Practical constraints (mobile compatibility, fallbacks) ensure production-readiness. Pseudocode testing prevents wasted implementation effort.

---

## Technology Stack

**MANDATED TECHNOLOGIES** (from hackathon requirements):

### Frontend
- **Framework**: Docusaurus 3.x (static site generator for book)
- **UI Library**: React 18+ (for custom components)
- **Chatbot UI**: OpenAI ChatKit (or custom React component matching ChatKit patterns)
- **Styling**: Docusaurus built-in themes + custom CSS modules
- **State Management**: React Context for user profile (from Better-Auth)

### Backend
- **Framework**: FastAPI 0.100+ (Python 3.11+)
- **AI/Agents**: OpenAI Agents SDK (for RAG orchestration)
- **Authentication**: Better-Auth (signup/signin with quiz)
- **Database**: Neon Postgres (user profiles, quiz responses, chapter metadata)
- **Vector DB**: Qdrant Cloud (book chunk embeddings, translation cache)
- **Translation**: LiteLLM with Groq backend (fast, cheap Urdu translation)

### Development Tools
- **Spec Framework**: Spec-Kit Plus (user stories, templates, ADRs)
- **Code Generation**: Claude Code subagents and skills
- **Version Control**: Git with feature branches (`###-feature-name` pattern)
- **CI/CD**: GitHub Actions (Pages deployment), Vercel (FastAPI)

### Course-Specific Technologies (Content)
- **Robotics Framework**: ROS 2 Humble (Ubuntu 22.04)
- **Simulation**: Gazebo Harmonic, Unity ML-Agents, NVIDIA Isaac Sim
- **Hardware**: Jetson Orin Nano, Intel RealSense D435i
- **AI Models**: OpenAI Whisper (speech-to-text), GPT-4 (planning), CLIP (vision)

---

## Development Workflow

### Phase 0: Spec-Kit Plus Planning
1. **Generate Specs**: Use `/sp.specify` to create feature specs with user stories
2. **Architectural Planning**: Use `/sp.plan` to generate implementation plan with ADRs
3. **Task Breakdown**: Use `/sp.tasks` to create dependency-ordered task list
4. **Validation**: Use `/sp.analyze` to check cross-artifact consistency

### Phase 1: Subagent Orchestration
1. **Launch Parallel Agents**:
   - `BookWriterAgent`: Generate chapter drafts from course modules
   - `DocusaurusSetupAgent`: Initialize site structure with custom plugins
   - `RAGPipelineAgent`: Set up FastAPI backend with Qdrant embedding
   - `BetterAuthAgent`: Integrate signup/signin with quiz form
2. **Skill Creation**:
   - `TranslateToUrduSkill`: Reusable translation with Qdrant caching
   - `HardwarePersonalizerSkill`: Query user profile and return content variants

### Phase 2: Integration & Testing
1. **Embed RAG Chatbot**: Integrate ChatKit component into all 13 chapters
2. **Add Personalization Buttons**: Per-chapter "Personalize Content" fetching user profile
3. **Add Translation Buttons**: Per-chapter "Translate to Urdu" with caching
4. **Test Assessments**: Verify capstone project (voice-command humanoid) is queryable via chatbot

### Phase 3: Deployment & Verification
1. **Deploy Docusaurus**: GitHub Actions workflow to Pages (`main` branch → gh-pages)
2. **Deploy FastAPI**: Vercel serverless functions (or Railway/Render)
3. **Verification Checklist** (see Governance section):
   - [ ] All 13 chapters render correctly (mobile + desktop)
   - [ ] RAG chatbot answers course questions accurately
   - [ ] Personalization adapts content based on quiz responses
   - [ ] Urdu translation works with caching (< 2s response time)
   - [ ] Signup/signin flow captures hardware/software background
   - [ ] No broken links or missing images
   - [ ] 250/250 points achieved (100 base + 150 bonus)

### Prompt History Records (PHR)
- **MUST** create PHR after every user interaction via `/sp.phr`
- **Route by stage**:
  - `constitution` → `history/prompts/constitution/`
  - Feature stages (`spec`, `plan`, `tasks`, `red`, `green`, `refactor`, `explainer`, `misc`) → `history/prompts/<feature-name>/`
  - `general` → `history/prompts/general/`
- **Filename format**: `<ID>-<slug>.<stage>.prompt.md` (e.g., `001-rag-setup.plan.prompt.md`)

### Architecture Decision Records (ADR)
- **Trigger**: Suggest ADR when decision meets all three criteria:
  1. **Impact**: Long-term consequences (framework, data model, API, security, platform)
  2. **Alternatives**: Multiple viable options considered
  3. **Scope**: Cross-cutting, influences system design
- **Suggestion Format**: "📋 Architectural decision detected: [brief]. Document? Run `/sp.adr [title]`"
- **NEVER** auto-create ADRs—require user consent
- **Group related decisions**: E.g., "Tech Stack Selection" ADR covers Docusaurus + FastAPI + Qdrant together

---

## Governance

### Amendment Procedure
1. **Propose Change**: Document reasoning in GitHub issue or Slack
2. **Impact Analysis**: Run `/sp.analyze` to check template consistency
3. **Version Bump**: Semantic versioning (MAJOR.MINOR.PATCH)
4. **Update Templates**: Propagate changes to `plan-template.md`, `spec-template.md`, `tasks-template.md`
5. **PHR Creation**: Document amendment via `/sp.phr --stage constitution`

### Versioning Policy
- **MAJOR** (x.0.0): Backward-incompatible principle removals or redefinitions (e.g., removing "Spec-Driven" principle)
- **MINOR** (0.x.0): New principle/section added or materially expanded guidance (e.g., adding "Security by Default" principle)
- **PATCH** (0.0.x): Clarifications, wording fixes, typo corrections (no semantic change)

### Compliance Review
- **Pre-Commit Hook**: Run constitution checks before committing code
- **PR Template**: Include checklist verifying adherence to principles
- **Quarterly Reviews**: Re-evaluate principles against project outcomes

### Verification Checklist (Hackathon Submission)

**Base Points (100/100)**:
- [ ] Docusaurus site deployed to GitHub Pages with 13 chapters (40 points)
- [ ] RAG chatbot embedded in book using OpenAI Agents SDK + Qdrant (40 points)
- [ ] Hardware guides with accurate prices ($700 Edge Kit, $2k+ Workstation) (20 points)

**Bonus Points (150/150)**:
- [ ] Claude Code subagents used (BookWriterAgent, RAGEmbeddingAgent, etc.) (40 points)
- [ ] Reusable skills created (TranslateToUrduSkill, HardwarePersonalizerSkill) (10 points)
- [ ] Better-Auth signup/signin with hardware/software quiz (40 points)
- [ ] Per-chapter personalization buttons fetching user profile from Neon (30 points)
- [ ] Per-chapter Urdu translation buttons using LiteLLM/Groq (30 points)

**Total**: 250/250 points

---

**Version**: 1.0.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-09
