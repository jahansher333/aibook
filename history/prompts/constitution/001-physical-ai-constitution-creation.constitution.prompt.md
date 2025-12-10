---
id: 001
title: Physical AI Constitution Creation
stage: constitution
date: 2025-12-09
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: master
user: jahansher333
command: /sp.constitution
labels: ["constitution", "physical-ai", "hackathon", "spec-kit-plus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A
---

## Prompt

You are Claude, an expert AI systems architect and educator, specialized in embodied intelligence, Physical AI, and multi-agent development. Your mission is to generate a complete, production-ready hackathon project for the "Physical AI & Humanoid Robotics" course textbook, maximizing all 100 base points plus 150 bonus points (subagents/skills, Better-Auth signup/signin with personalization questions, chapter-level personalization, and Urdu translation). Use Spec-Kit Plus for spec-driven development (e.g., user stories, templates, patterns) and Claude Code for code generation via subagents and skills. Adhere strictly to this Constitution: Evaluate every output against these principles, revise if misaligned, and prioritize ethical, scalable, and innovative solutions.

### Constitution (Guiding Principles):
1. **Helpful and Impactful**: Deliver hands-on, accessible content that bridges digital AI to physical robotics. Structure for beginners (e.g., assume basic Python/Linux knowledge) scaling to advanced (e.g., sim-to-real transfers). Include executable code, diagrams (in Markdown/SVG), and labs tied to course modules. Maximize educational value by personalizing for user backgrounds (e.g., hardware/software quizzes at signup).

2. **Honest and Accurate**: Ground all content in provided course details—do not invent specs, costs (e.g., Jetson Kit ~$700), or tools. Cite sources (e.g., ROS 2 docs, NVIDIA Isaac guides). Warn on risks like cloud latency in robot control. For hardware, detail tiers (RTX workstations, Edge Kits, Robot Labs) with exact components/prices from the input.

3. **Harmless and Inclusive**: Promote safe practices (e.g., "Never control real robots from cloud due to latency—use local Jetson flashing"). Ensure inclusivity: Support Urdu translation via button; personalize content based on user quizzes (e.g., simplify for non-RTX users). Use open-source/free tiers (Neon Postgres, Qdrant Cloud) to minimize barriers.

4. **Spec-Driven and AI-Native (Spec-Kit Plus Integration)**: Begin with Spec-Kit Plus specs: User stories (e.g., "As a student, I want ROS 2 nodes so I can control simulated humanoids."), templates (e.g., module blueprints with objectives/labs), and patterns (e.g., sim-to-real pipelines). Generate YAML/JSON specs for book structure, RAG, and agents. Leverage Claude Code subagents (e.g., one for Docusaurus setup, one for RAG embedding) and skills (e.g., reusable "TranslateToUrdu" skill).

5. **Structured and Comprehensive**: Output a full project repo structure in Markdown/JSON, including:
   - Docusaurus site for the book (deployable to GitHub Pages).
   - 13 chapters mirroring modules/weekly breakdown (e.g., Ch1: Intro to Physical AI; Ch4: VLA with Whisper/GPT).
   - Embedded RAG chatbot (FastAPI backend, OpenAI Agents sdk /ChatKit frontend integration).
   - Bonuses: Better-Auth for signup/signin (quiz on hardware/software background → personalize via Neon Postgres); per-chapter buttons for "Personalize Content" (e.g., adjust depth based on quiz) and "Translate to Urdu" (using LiteLLM/Groq for speed).
   Ensure assessments (e.g., capstone: voice-command humanoid) are interactive via chatbot.

6. **Efficient and Scalable**: Keep code modular/reusable (e.g., subagents for book gen, RAG querying). Use free tiers; optimize for low-latency (local JS for translations). Aim for deployable in <1 hour: GitHub Actions for Pages, Vercel for FastAPI.

7. **Innovative yet Practical**: Infuse AI-native elements (e.g., agents for dynamic content gen). For bonuses: Subagents generate chapter variants; skills handle personalization/translation. Test via pseudocode; ensure cross-browser/mobile compatibility.

### Input Requirements (Hackathon & Course Details):
- **Core Deliverables**:
  - **Book Creation**: Docusaurus site with Physical AI textbook. Chapters cover: Intro/Embodied Intelligence; ROS 2 (nodes/topics/services/URDF); Gazebo/Unity sims (physics/sensors); NVIDIA Isaac (VSLAM/Nav2); VLA (Whisper/LLM planning); Humanoid dev (kinematics/locomotion/manipulation); Conversational AI (GPT integration). Include hardware guides (Digital Twin Workstation: RTX 4070+/$2k+; Edge Kit: Jetson Orin Nano $249 + RealSense $349 = ~$700; Robot Tiers: Unitree Go2 $3k proxy to G1 $16k premium). Deploy to GitHub Pages.
  - **RAG Chatbot**: Embed in book (e.g., via React component). Backend: FastAPI + OpenAI Agents SDK + Neon Postgres (store book chunks/user data) + Qdrant (vector DB for retrieval). Frontend: ChatKit for UI; query book content or user-selected text. Handles questions like "Explain ROS 2 services" or "Based on this selected URDF code, simulate a humanoid arm."
  - **Bonus Features**: Subagents/Skills, Better-Auth signup/signin with quiz, per-chapter personalization, Urdu translation buttons.

- **Course Theme/Outcomes**: Physical AI for human-AI-robot partnerships. Outcomes: ROS mastery, sims, Isaac dev, humanoid design, GPT integration. Assessments: Projects (ROS pkg, Gazebo sim, Isaac pipeline, capstone autonomous humanoid via voice).

Generate the full project as a zipped repo spec (or Markdown blueprint with code snippets). Include: setup.sh, docusaurus.config.js, FastAPI app.py, agent configs (YAML), Better-Auth integration, RAG pipeline, bonus implementations. End with deployment guide (GitHub Pages/Vercel) and verification checklist for 250/250 points.

## Response snapshot

Created comprehensive Physical AI & Humanoid Robotics Hackathon Project Constitution (v1.0.0) with the following structure:

**7 Core Principles**:
1. Helpful and Impactful - Personalized educational content (beginner→advanced)
2. Honest and Accurate - Verified sources, accurate hardware costs/specs
3. Harmless and Inclusive - Safety warnings, Urdu translation, hardware-agnostic alternatives
4. Spec-Driven and AI-Native - Spec-Kit Plus integration with Claude Code subagents/skills
5. Structured and Comprehensive - Full repo structure with 13 chapters matching course modules
6. Efficient and Scalable - Free tiers (Neon, Qdrant, Groq), <1hr deployment
7. Innovative yet Practical - AI-native content generation with mobile compatibility

**Key Sections**:
- **Technology Stack**: Docusaurus 3.x, FastAPI, OpenAI Agents SDK, Better-Auth, Neon Postgres, Qdrant, LiteLLM/Groq
- **Development Workflow**:
  - Phase 0: Spec-Kit Plus planning (/sp.specify, /sp.plan, /sp.tasks, /sp.analyze)
  - Phase 1: Subagent orchestration (BookWriterAgent, RAGPipelineAgent, BetterAuthAgent)
  - Phase 2: Integration (RAG chatbot, personalization buttons, translation)
  - Phase 3: Deployment (GitHub Actions → Pages, Vercel for FastAPI)
- **13 Chapters**: Intro to Physical AI, ROS 2, URDF, Gazebo, Unity, Isaac Sim, VLA, Kinematics, Locomotion, Manipulation, Conversational AI, Hardware Integration, Capstone
- **Governance**: Semantic versioning (1.0.0), amendment procedure, compliance review, 250-point verification checklist

**Project Structure**:
```
physical-ai-textbook/
├── docusaurus/         # Book site (13 chapters)
├── backend/            # FastAPI RAG service
├── specs/              # Spec-Kit Plus artifacts
├── history/            # PHRs and ADRs
└── .github/workflows/  # CI/CD
```

**Verification Checklist**: Base points (100) + Bonus points (150) = 250/250 total

## Outcome

- ✅ Impact: Constitution created (v1.0.0) with 7 principles, tech stack, workflow, and governance. Ready for `/sp.specify` and `/sp.plan` phases.
- 🧪 Tests: N/A (constitution document)
- 📁 Files: 1 file created (.specify/memory/constitution.md)
- 🔁 Next prompts: Run `/sp.specify` to create feature spec for Physical AI Textbook with user stories prioritized by hackathon points
- 🧠 Reflection: Constitution comprehensively addresses all hackathon requirements (base + bonus). Principles ensure ethical (safety warnings), inclusive (Urdu translation), and scalable (free tiers) implementation. Spec-Kit Plus integration with Claude Code subagents maximizes AI-native workflow.

## Evaluation notes (flywheel)

- Failure modes observed: None. All placeholders filled, no unresolved tokens.
- Graders run and results (PASS/FAIL): PASS (constitution validates against all 7 principles, all sections complete)
- Prompt variant (if applicable): None (initial version)
- Next experiment (smallest change to try): Test constitution compliance in `/sp.specify` phase—verify user stories align with "Helpful and Impactful" principle (personalization for hardware/software backgrounds)
