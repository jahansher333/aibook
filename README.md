# Physical AI & Humanoid Robotics Textbook

[![Deploy to GitHub Pages](https://github.com/your-org/physical-ai-textbook/actions/workflows/deploy.yml/badge.svg)](https://github.com/your-org/physical-ai-textbook/actions/workflows/deploy.yml)
[![License: CC BY-SA 4.0](https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-sa/4.0/)
[![Code License: MIT](https://img.shields.io/badge/Code%20License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Comprehensive interactive textbook teaching **ROS 2**, **Gazebo/Unity simulation**, **NVIDIA Isaac Sim**, and **Vision-Language-Action (VLA)** models for humanoid robots.

## Features

- 🤖 **13 Comprehensive Chapters**: Week-by-week progression from ROS 2 basics to capstone project
- 📝 **Interactive MCQs**: Instant feedback with explanations and citations
- 💻 **Hands-on Labs**: Executable code for simulation and hardware deployment (Isaac Sim, Gazebo)
- 💰 **Hardware Guides**: Three budget tiers (Economy $700, Proxy $3,850, Cloud $205/quarter)
- 🌙 **Dark Mode**: Fully responsive with light/dark theme toggle
- 🌍 **i18n Ready**: Stub for future Urdu translation (RTL support)
- 📊 **Mermaid Diagrams**: Flowcharts, sequence diagrams, state machines

## Quick Start

### Prerequisites

- **Node.js 20.x LTS** ([Download](https://nodejs.org/))
- **Git**

### Local Development

```bash
# Clone repository
git clone https://github.com/your-org/physical-ai-textbook.git
cd physical-ai-textbook/docusaurus

# Install dependencies
npm install

# Start development server
npm run start
```

Open [http://localhost:3000/](http://localhost:3000/) to view the site with live reload.

### Build for Production

```bash
cd docusaurus
npm run build
npm run serve
```

Site generated in `build/` folder, served at [http://localhost:3000/](http://localhost:3000/).

## Deployment

### GitHub Pages (Automated)

1. **Enable GitHub Pages**:
   - Go to repo Settings > Pages
   - Source: "GitHub Actions"

2. **Configure URL** in `docusaurus/docusaurus.config.ts`:
   ```typescript
   url: 'https://your-username.github.io',
   baseUrl: '/physical-ai-textbook/',
   organizationName: 'your-username',
   projectName: 'physical-ai-textbook',
   ```

3. **Push to main branch**:
   ```bash
   git add .
   git commit -m "feat: deploy Physical AI textbook"
   git push origin main
   ```

GitHub Actions automatically builds and deploys to https://your-username.github.io/physical-ai-textbook/

### Vercel (Alternative)

```bash
npm i -g vercel
cd docusaurus
vercel --prod
```

Or connect GitHub repo in [Vercel Dashboard](https://vercel.com/dashboard) for auto-deploy on push.

## Project Structure

```
physical-ai-textbook/
├── docusaurus/                      # Docusaurus site root
│   ├── docs/                        # Documentation content
│   │   ├── intro.md                 # Landing page
│   │   ├── ch01-physical-ai-intro/  # Chapter 1
│   │   ├── ch02-ros2-fundamentals/  # Chapter 2
│   │   ├── ... (ch03-ch13)          # Chapters 3-13
│   │   └── hardware-requirements.md # Hardware guide
│   ├── src/
│   │   ├── components/              # Custom React components
│   │   │   ├── MCQ/                 # MCQ component with instant feedback
│   │   │   └── CollapsibleLab/      # Collapsible code sections
│   │   ├── css/
│   │   │   └── custom.css           # Global styles, dark mode
│   │   └── pages/
│   │       └── index.tsx            # Custom landing page
│   ├── static/
│   │   ├── img/                     # Images (robot diagrams, hardware photos)
│   │   └── data/
│   │       └── hardware-tiers.yaml  # Hardware tier configurations
│   ├── i18n/
│   │   └── ur/                      # Urdu translation stub (future)
│   ├── docusaurus.config.ts         # Site configuration
│   ├── sidebars.ts                  # Sidebar navigation (13 chapters)
│   └── package.json                 # Dependencies
├── .github/
│   └── workflows/
│       └── deploy.yml               # GitHub Actions CI/CD pipeline
├── specs/                           # Spec-Kit Plus artifacts
│   └── 1-physical-ai-textbook/
│       ├── spec.md                  # Feature specification
│       ├── plan.md                  # Implementation plan
│       ├── tasks.md                 # Task breakdown
│       ├── research.md              # Technology decisions
│       ├── data-model.md            # Entity schemas
│       ├── quickstart.md            # Developer guide
│       └── contracts/               # API contracts (YAML)
└── README.md                        # This file
```

## Technology Stack

| Category          | Technology                | Version   | Purpose                                   |
|-------------------|---------------------------|-----------|-------------------------------------------|
| Runtime           | Node.js                   | 20.x LTS  | JavaScript runtime                        |
| Framework         | Docusaurus                | 3.5.2+    | Static site generator                     |
| UI Library        | React                     | 18+       | Component-based UI                        |
| Content Format    | MDX                       | 3.x       | Markdown + JSX                            |
| Diagrams          | Mermaid                   | 10.x      | Declarative diagram syntax                |
| Syntax Highlighting | Prism React Renderer    | 2.x       | Code highlighting (Python, Bash, YAML)    |
| Testing           | Jest + React Testing Lib  | 29.x/14.x | Component unit tests                      |
| CI/CD             | GitHub Actions            | N/A       | Automated build & deploy                  |
| Hosting           | GitHub Pages              | N/A       | Free static hosting with CDN              |

## Chapter Overview

### Fundamentals (Weeks 1-3)
1. **Introduction to Physical AI** - Embodied intelligence, humanoid architectures
2. **ROS 2 Fundamentals** - Nodes, topics, services, actions
3. **Robot Modeling** - URDF, TF2, Xacro, visualization

### Simulation (Weeks 4-6)
4. **Gazebo Simulation** - Physics engines, sensor plugins, world files
5. **Unity Simulation** - ML-Agents integration, high-fidelity rendering
6. **NVIDIA Isaac Sim** - VSLAM, Nav2 path planning, GPU acceleration

### Advanced AI (Weeks 7-9)
7. **Vision-Language-Action Models** - Whisper, GPT-4, CLIP integration
8. **Humanoid Kinematics** - Forward/inverse kinematics, Jacobian, DH parameters
9. **Locomotion** - Bipedal walking, ZMP control, trajectory optimization

### Integration (Weeks 10-13)
10. **Manipulation** - Grasping, force control, motion planning with MoveIt2
11. **Conversational AI Integration** - GPT-4 for natural language robot commands
12. **Hardware Integration** - Jetson Orin Nano setup, RealSense D435i, sensor fusion
13. **Capstone Project** - Voice-command autonomous humanoid (Whisper + GPT-4 + ROS 2)

## Hardware Requirements

Three budget tiers for different learning contexts:

### Economy Tier (~$700)
- Jetson Orin Nano Developer Kit ($249)
- Intel RealSense D435i ($349)
- Accessories ($102: power supply, microSD, cooling)
- **Best for**: Solo learners, students, DIY projects

### Proxy Tier (~$3,850)
- Unitree Go2 Quadruped Robot ($3,000)
- Jetson Xavier NX ($500)
- Accessories ($350)
- **Best for**: Research labs, university courses

### Cloud OpEx Tier (~$205/quarter)
- AWS EC2 g4dn.xlarge ($192/quarter for 40 hours)
- EBS Storage 100GB ($10/quarter)
- Data Transfer ($3/quarter)
- **Best for**: Cloud-only learners, institutions without CapEx budgets

See [Hardware Requirements](https://your-username.github.io/physical-ai-textbook/docs/hardware-requirements) for detailed comparisons.

## Development

### Adding a New Chapter

1. Create chapter folder:
   ```bash
   mkdir -p docs/ch14-new-topic
   ```

2. Create `index.md` with frontmatter:
   ```markdown
   ---
   id: ch14
   title: "New Topic Title"
   sidebar_position: 15
   week: 14
   objectives:
     - "Learning outcome 1"
     - "Learning outcome 2"
   tags: [topic, keywords]
   ---

   # New Topic Title

   ## Objectives
   ## Theory
   ## Hands-on Lab
   ## Assessment
   ```

3. Update `sidebars.ts` to include new chapter
4. Verify in dev server: `npm run start`

### Running Tests

```bash
cd docusaurus
npm run test
```

Tests MCQ React components with Jest + React Testing Library.

### Code Quality

```bash
# Lint Markdown
npm run lint

# Format code with Prettier
npm run format

# Clear cache (if build issues)
npm run clear
```

## Performance

**Lighthouse Scores (Target)**:
- Performance: >90
- Accessibility: >90 (WCAG 2.1 AA compliant)
- Best Practices: >90
- SEO: >90

**Build Metrics**:
- Build time: <5 minutes (GitHub Actions)
- Bundle size: <10 MB
- Time to Interactive: <5s on 4G connection

## Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improve-chapter-2`)
3. Commit changes (`git commit -m 'feat: improve ROS 2 examples'`)
4. Push to branch (`git push origin feature/improve-chapter-2`)
5. Open a Pull Request

See [CONTRIBUTING.md](CONTRIBUTING.md) for detailed guidelines.

## License

### Content License
Educational content (Markdown files, diagrams) licensed under [Creative Commons Attribution-ShareAlike 4.0 International (CC BY-SA 4.0)](https://creativecommons.org/licenses/by-sa/4.0/).

### Code License
Code examples and React components licensed under [MIT License](https://opensource.org/licenses/MIT).

## Acknowledgments

- **ROS 2**: Open Robotics for ROS 2 Humble documentation
- **NVIDIA**: Isaac Sim platform and documentation
- **OpenAI**: Whisper, GPT-4 APIs for VLA models
- **Docusaurus**: Meta for the static site generator framework

## Support

- **Issues**: [GitHub Issues](https://github.com/your-org/physical-ai-textbook/issues)
- **Discussions**: [GitHub Discussions](https://github.com/your-org/physical-ai-textbook/discussions)
- **Email**: support@physicalai.edu (institutional inquiries)

## Citation

If you use this textbook in your research or teaching, please cite:

```bibtex
@book{physicalai2025,
  title={Physical AI and Humanoid Robotics Textbook},
  author={Physical AI Education Team},
  year={2025},
  publisher={Open Educational Resources},
  url={https://github.com/your-org/physical-ai-textbook},
  note={Interactive textbook covering ROS 2, Isaac Sim, and VLA models}
}
```

---

**Live Site**: https://your-username.github.io/physical-ai-textbook/

**Built with**: [Docusaurus 3](https://docusaurus.io/) | **Spec Framework**: [Spec-Kit Plus](https://github.com/panaversity/spec-kit-plus/)
