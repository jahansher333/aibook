---
id: 002
title: Physical AI Textbook Implementation Plan
stage: plan
date: 2025-12-09
surface: agent
model: claude-opus-4-5-20251101
feature: 1-physical-ai-textbook
branch: 1-physical-ai-textbook
user: jahansher333
command: /sp.plan
labels: ["plan", "docusaurus", "architecture", "nextjs", "mermaid", "github-actions"]
links:
  spec: specs/1-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-physical-ai-textbook/plan.md
  - specs/1-physical-ai-textbook/research.md
  - specs/1-physical-ai-textbook/data-model.md
  - specs/1-physical-ai-textbook/quickstart.md
  - specs/1-physical-ai-textbook/contracts/mcq-component.yaml
  - specs/1-physical-ai-textbook/contracts/chapter-frontmatter.yaml
  - specs/1-physical-ai-textbook/contracts/mermaid-diagrams.yaml
tests:
  - Constitution Check (all 7 principles passed)
  - Technology stack validation (Node 20, Docusaurus 3.x, MDX, Mermaid)
---

## Prompt

Docusaurus Textbook for Physical AI & Humanoid Robotics
Use specs from previous phase.
Success criteria:
- Full Docusaurus v3 classic site scaffold with nextjs full functionality
- Auto-generated sidebar with 13 chapters + Hardware + Capstone
- MD-ready chapter template with code blocks, mermaid diagrams, collapsible labs
- GitHub Actions workflow or vercel that builds and deploys to GitHub Pages on push or vercel
- i18n stub for future Urdu (bonus)
- Dark mode + mobile friendly
Tech stack: Node 20, Docusaurus 3,nextjs, MX, Mermaid, GitHub Pages or vercel
Tasks must be numbered, prioritized [P1][P2][P3], and parallelizable
What the agent does: Output complete plan.md + tasks.md + Mermaid architecture diagram + exact GitHub Actions YAML. Ready for instant /sp.implement.

## Response snapshot

Created comprehensive implementation plan for Physical AI & Humanoid Robotics Textbook with:

**Complete plan.md** covering:
- Summary: Docusaurus 3.x site with 13 chapters, MCQ components, Mermaid diagrams, GitHub Actions deployment
- Technical Context: Node.js 20, Docusaurus 3.5.2, React 18, MDX 3.x, Mermaid, Next.js 14 integration
- Constitution Check: ✅ All 7 principles passed (Helpful, Honest, Harmless, Spec-Driven, Structured, Efficient, Innovative)
- Project Structure: docusaurus/ folder with docs/, src/components/, static/, i18n/, .github/workflows/
- Complexity Tracking: No violations (single project, standard React patterns, minimal dependencies)

**Phase 0: Research** (5 key decisions):
1. **Why Docusaurus over GitBook/MkDocs**: React integration, MDX support, Next.js compatibility, open-source (MIT), plugin ecosystem
2. **Client-Side MCQs**: React useState for instant feedback, localStorage for progress, no backend (static site constraint)
3. **Mermaid Integration**: @docusaurus/theme-mermaid plugin, code fence syntax (```mermaid), SVG output at build time
4. **GitHub Actions vs Vercel**: GitHub Actions primary (free 2000 min/month), GitHub Pages hosting, Vercel as fallback
5. **i18n Stub for Urdu**: Docusaurus i18n plugin with empty ur/ locale, RTL support, future LiteLLM/Groq translation

**Phase 1: Design & Contracts**:
- **data-model.md**: 4 entities (Chapter, MCQ, HardwareTier, UserProgress with localStorage schema)
- **contracts/** directory:
  - mcq-component.yaml: MCQ React component API (props: question, options, correctIndex, explanation)
  - chapter-frontmatter.yaml: MDX frontmatter schema (id, title, objectives, tags)
  - mermaid-diagrams.yaml: Supported types (flowchart, sequence, class, stateDiagram)
- **quickstart.md**: Local dev setup (npm install/start/build), adding chapters, troubleshooting, deployment

**Mermaid Architecture Diagrams**:
1. System Architecture: Content Layer → Component Layer → Build Layer → Deploy Layer → User Layer
2. Chapter Content Flow: Sequence diagram (Student → Browser → MDX → MCQ Component → localStorage)
3. CI/CD Deployment: Flowchart (Push to main → GitHub Actions → Build → Test → Deploy to Pages)

**Exact GitHub Actions YAML** (.github/workflows/deploy.yml):
- Triggers: push to main, pull_request, workflow_dispatch
- Build job: Checkout → Setup Node 20 → npm ci → npm run build → Lighthouse CI → Upload artifact
- Deploy job: Deploy to GitHub Pages (peaceiris/actions-gh-pages@v3) on main branch only
- Optional Vercel deployment (commented out, requires VERCEL_TOKEN secret)

**Technology Stack Finalized**:

| Category       | Technology            | Version   | Rationale                                  |
|----------------|-----------------------|-----------|--------------------------------------------|
| Runtime        | Node.js               | 20.x LTS  | Required by Docusaurus 3.x, stable         |
| Framework      | Docusaurus            | 3.5.2+    | Static site generator, MDX, plugin ecosystem |
| UI Library     | React                 | 18+       | Component-based, required by Docusaurus    |
| Content Format | MDX                   | 3.x       | Markdown + JSX, allows React in content    |
| Diagrams       | Mermaid               | 10.x      | Declarative syntax, SVG output             |
| CI/CD          | GitHub Actions        | N/A       | Free for public repos, 2000 min/month      |
| Deployment     | GitHub Pages (primary)| N/A       | Free static hosting, CDN, custom domain    |
| Deployment (alt)| Vercel (fallback)    | N/A       | Unlimited bandwidth, Next.js optimized     |
| i18n           | Docusaurus i18n Plugin| N/A       | Built-in, RTL for Urdu, locale routing     |

**Performance Targets**:
- Time to Interactive (TTI): <5s on 4G
- First Contentful Paint (FCP): <2s
- Lighthouse Performance: >90
- Build time: <5 minutes on GitHub Actions
- Bundle size: <10 MB

**Project Structure**:
```
physical-ai-textbook/
├── docusaurus/
│   ├── docs/ (13 chapters + hardware page)
│   ├── src/components/ (MCQ, CollapsibleLab, LandingAnimation)
│   ├── static/ (images, data/hardware-tiers.yaml)
│   ├── i18n/ur/ (Urdu stub)
│   ├── docusaurus.config.js
│   ├── sidebars.js
│   └── package.json
├── .github/workflows/deploy.yml
└── README.md
```

**Ready for /sp.tasks**: All technical decisions finalized, architecture designed, contracts specified, deployment pipeline defined.

**ADR Suggestions**:
- `/sp.adr why-docusaurus-over-gitbook` - Framework choice rationale
- `/sp.adr client-side-mcqs-no-backend` - Static site approach for assessments
- `/sp.adr github-actions-primary-deployment` - CI/CD pipeline selection

## Outcome

- ✅ Impact: Complete implementation plan with 7 artifacts (plan.md, research.md, data-model.md, quickstart.md, 3 contract YAMLs). Architecture diagrams, GitHub Actions YAML, technology stack finalized. Ready for /sp.tasks phase.
- 🧪 Tests: Constitution Check passed (7/7 principles). Technology choices validated against requirements (Node 20 ✓, Docusaurus 3 ✓, MDX ✓, Mermaid ✓, GitHub Pages ✓). Performance targets defined (TTI <5s, Lighthouse >90).
- 📁 Files: 7 files created (plan.md, research.md, data-model.md, quickstart.md, contracts/*.yaml)
- 🔁 Next prompts: Run `/sp.tasks` to generate numbered, prioritized task list with [P1][P2][P3] labels and parallelization opportunities. Consider ADRs for technology decisions.
- 🧠 Reflection: Plan aligns with all constitution principles. Research phase resolved 5 key questions (Docusaurus choice, MCQ implementation, Mermaid integration, deployment strategy, i18n architecture). Data model covers 4 entities (Chapter, MCQ, HardwareTier, UserProgress). Contracts specify React component APIs, MDX frontmatter schemas, Mermaid diagram types. Quickstart provides local dev workflow. GitHub Actions YAML ready for immediate deployment. Zero ambiguity remaining.

## Evaluation notes (flywheel)

- Failure modes observed: None. All technical decisions backed by research, alternatives considered, rationale documented.
- Graders run and results (PASS/FAIL): PASS (Constitution Check 7/7, technology stack validated, contracts complete, deployment pipeline defined)
- Prompt variant (if applicable): None (initial version)
- Next experiment (smallest change to try): In `/sp.tasks` phase, validate task parallelization opportunities - identify tasks that can run concurrently (e.g., chapter template creation + MCQ component development + Mermaid diagram design) vs. sequential dependencies (e.g., Docusaurus setup → chapter creation). Test build time <5 minutes target with realistic content estimates.
