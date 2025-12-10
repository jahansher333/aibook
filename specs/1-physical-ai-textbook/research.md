# Research: Physical AI & Humanoid Robotics Textbook

**Feature**: Physical AI Textbook Platform
**Date**: 2025-12-09
**Phase**: Phase 0 - Technology Research & Decision Making

## Research Questions & Decisions

### RQ1: Why Docusaurus over GitBook or MkDocs?

**Decision**: Docusaurus 3.x Classic preset

**Rationale**:
1. **React Integration**: MDX allows React components (MCQ, animations) in Markdown without build complexity. GitBook limits interactivity; MkDocs uses Python/Jinja2 (less familiar for web devs).
2. **Performance**: Docusaurus generates static HTML (JAMstack), loads only needed JS. GitBook's SaaS has slower cold starts; MkDocs lacks modern optimization.
3. **Next.js Support**: Docusaurus 3.x integrates Next.js for React Server Components, future-proofing for RAG chatbot (constitution bonus feature).
4. **Open Source**: Docusaurus is MIT-licensed, community-driven (Meta). GitBook's self-hosted version is limited; SaaS has paywalls.
5. **Plugin Ecosystem**: Mermaid, i18n, algolia search built-in. MkDocs requires manual plugin config; GitBook plugins are proprietary.

**Alternatives Considered**:
- **GitBook**: Excellent UI but SaaS lock-in, limited interactivity, costly for teams ($99/month for Pro).
- **MkDocs (Material theme)**: Great for Python projects but lacks React integration, MDX support. Python-based tooling less aligned with Node.js ecosystem.
- **Nextra (Next.js + MDX)**: More flexible but requires custom config for sidebar, search, versioning (Docusaurus provides out-of-box).

**References**:
- [Docusaurus 3 Documentation](https://docusaurus.io/docs)
- [MDX Documentation](https://mdxjs.com/)
- [GitBook vs Docusaurus Comparison](https://stackshare.io/stackups/docusaurus-vs-gitbook)

---

### RQ2: How to Implement Interactive MCQs in Static Site?

**Decision**: React component with useState for client-side state, no backend

**Rationale**:
1. **Static Site Constraint**: GitHub Pages serves static HTML. MCQ state (selected answer, score) stored in browser memory (useState) or localStorage for persistence across sessions.
2. **Instant Feedback**: On answer selection, compare to correct answer (hardcoded in component props), highlight green/red, show explanation.
3. **Accessibility**: Use semantic HTML (radio inputs for options), ARIA labels for screen readers, keyboard navigation (arrow keys, Enter to submit).
4. **Scalability**: Each chapter imports MCQ component with question data as props. No API calls = no rate limits, instant load.

**Alternatives Considered**:
- **Backend API**: Requires server (Vercel serverless, Firebase) for storing scores. Adds complexity, cost, latency. Not needed for MVP (localStorage sufficient for personal progress tracking).
- **Quiz Libraries** (react-quiz-component): Heavyweight (bundle size ~50KB), limited customization for educational context. Custom component <10KB.

**Implementation Pattern**:
```jsx
// src/components/MCQ/index.tsx
import React, { useState } from 'react';

export default function MCQ({ question, options, correctIndex, explanation }) {
  const [selected, setSelected] = useState(null);
  const [showFeedback, setShowFeedback] = useState(false);

  const handleSubmit = () => {
    setShowFeedback(true);
  };

  return (
    <div className="mcq-container">
      <h3>{question}</h3>
      {options.map((opt, idx) => (
        <label key={idx} className={showFeedback && idx === correctIndex ? 'correct' : showFeedback && idx === selected ? 'incorrect' : ''}>
          <input type="radio" name="mcq" checked={selected === idx} onChange={() => setSelected(idx)} />
          {opt}
        </label>
      ))}
      <button onClick={handleSubmit} disabled={selected === null}>Submit</button>
      {showFeedback && (
        <div className="feedback">
          <p>{selected === correctIndex ? '✅ Correct!' : '❌ Incorrect'}</p>
          <p>{explanation}</p>
        </div>
      )}
    </div>
  );
}
```

**References**:
- [React Hooks Documentation](https://react.dev/reference/react/hooks)
- [WCAG 2.1 AA Form Accessibility](https://www.w3.org/WAI/WCAG21/Understanding/)
- [localStorage API](https://developer.mozilla.org/en-US/docs/Web/API/Window/localStorage)

---

### RQ3: Mermaid Diagram Integration Best Practices

**Decision**: @docusaurus/theme-mermaid plugin with code fence syntax

**Rationale**:
1. **Built-in Support**: Docusaurus 3.x includes Mermaid plugin, zero config needed beyond enabling in docusaurus.config.js.
2. **Markdown-Native**: Write diagrams as code fences (```mermaid), renders as SVG at build time. No runtime JS overhead.
3. **No External Dependencies**: No heavy JS libs (vis.js, D3.js), no iframes, no third-party services (Lucidchart, draw.io).
4. **Supported Types**: Flowcharts (ROS 2 node graphs), sequence diagrams (robot control flow), class diagrams (URDF structure), state machines (navigation states).

**Example Usage** (in docs/ch02-ros2-fundamentals/index.md):
````markdown
## ROS 2 Node Communication

```mermaid
graph LR
    A[Publisher Node] -->|/cmd_vel topic| B[Subscriber Node]
    B --> C[Robot Actuators]
```
````

**Configuration** (docusaurus.config.js):
```js
module.exports = {
  themes: ['@docusaurus/theme-mermaid'],
  markdown: {
    mermaid: true,
  },
};
```

**Alternatives Considered**:
- **PlantUML**: Requires Java runtime, harder to integrate in JS build. Mermaid is JS-native.
- **Lucidchart embeds**: Proprietary, breaks if service down, not version-controlled. PNG exports lose editability.
- **D3.js custom**: Requires JavaScript expertise, maintenance burden. Mermaid provides declarative syntax.

**Best Practices**:
- Keep diagrams simple (<10 nodes for flowcharts)
- Use descriptive labels (not A, B, C)
- Prefer flowcharts for architecture, sequence for protocols
- Test rendering in Docusaurus dev server before commit (mermaid.live for syntax validation)

**References**:
- [Docusaurus Mermaid Plugin](https://docusaurus.io/docs/markdown-features/diagrams)
- [Mermaid Syntax Guide](https://mermaid.js.org/intro/)
- [Mermaid Live Editor](https://mermaid.live/)

---

### RQ4: GitHub Actions vs. Vercel for Deployment?

**Decision**: GitHub Actions primary, Vercel fallback documented

**Rationale**:
1. **Cost**: GitHub Actions free (2000 minutes/month public repos), GitHub Pages free (1GB limit, 100GB soft bandwidth). Vercel free tier unlimited bandwidth but may throttle on high traffic.
2. **Simplicity**: One repo, one workflow file (.github/workflows/deploy.yml), deploys to gh-pages branch. No external account needed (uses GITHUB_TOKEN).
3. **Control**: Full access to build logs, cache Node modules (saves ~2 minutes), run Lighthouse CI checks before deploy.
4. **Educational Context**: GitHub Actions teaches CI/CD concepts. Students can fork repo, see workflow runs.
5. **Fallback**: If repo exceeds 1GB or wants custom domain with advanced features, Vercel documented as alternative (same build commands).

**GitHub Actions Workflow** (simplified):
```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 20
          cache: 'npm'
          cache-dependency-path: docusaurus/package-lock.json
      - run: cd docusaurus && npm ci
      - run: cd docusaurus && npm run build
      - uses: peaceiris/actions-gh-pages@v3
        if: github.ref == 'refs/heads/main'
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./docusaurus/build
```

**Alternatives Considered**:
- **Netlify**: Similar to Vercel but less JS-optimized. Vercel preferred for Next.js compatibility (if future Next.js features needed).
- **Cloudflare Pages**: Great CDN but GitHub Pages simpler for educational open-source project (no account setup, integrated with repo).
- **Self-hosted VPS**: Overkill for static site, adds maintenance burden (updates, security patches).

**Comparison Matrix**:

| Feature             | GitHub Actions + Pages | Vercel Free Tier | Netlify Free Tier |
|---------------------|------------------------|------------------|-------------------|
| Cost                | Free (2000 min)        | Free (unlimited) | Free (300 min)    |
| Bandwidth Limit     | 100GB soft limit       | Unlimited        | 100GB             |
| Build Time          | ~3-5 min (caching)     | ~2-3 min         | ~3-4 min          |
| Custom Domain       | Yes (CNAME)            | Yes (auto HTTPS) | Yes (auto HTTPS)  |
| Preview Deploys     | Manual (PR builds)     | Auto (per PR)    | Auto (per PR)     |
| CDN                 | GitHub CDN             | Vercel Edge      | Netlify CDN       |
| Best For            | Open-source projects   | Next.js apps     | JAMstack sites    |

**Decision**: GitHub Actions for MVP. Document Vercel as fallback if bandwidth/preview deploys needed.

**References**:
- [GitHub Actions Documentation](https://docs.github.com/en/actions)
- [Docusaurus Deployment Guide](https://docusaurus.io/docs/deployment)
- [Vercel Docusaurus Integration](https://vercel.com/guides/deploying-docusaurus-with-vercel)

---

### RQ5: i18n Architecture for Future Urdu Translation

**Decision**: Docusaurus i18n plugin with stub locale (ur)

**Rationale**:
1. **Future-Proof**: Constitution requires Urdu translation (bonus feature). i18n structure now prevents major refactor later.
2. **Docusaurus Native**: i18n plugin handles locale routing (/ur/ prefix), RTL layout, translation file management (JSON for UI, separate docs/ folder for content).
3. **Stub Implementation**: Create i18n/ur/ folder with empty JSON files. Future agents (TranslationAgent per constitution) populate with LiteLLM/Groq API calls.
4. **No Performance Cost**: Empty locale ignored in build (not included in output), adds ~1KB config overhead.
5. **RTL Support**: Docusaurus auto-applies RTL CSS for Urdu (direction: rtl), handles text alignment, scrollbar positioning.

**Configuration** (docusaurus.config.js):
```js
module.exports = {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'], // English + Urdu stub
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
      ur: {
        label: 'اردو', // Urdu script
        direction: 'rtl',
        htmlLang: 'ur-PK',
      },
    },
  },
};
```

**Directory Structure** (after translation):
```
docusaurus/
├── docs/                          # English content (default)
│   └── ch01-physical-ai-intro/
│       └── index.md
├── i18n/
│   └── ur/                        # Urdu translations (future)
│       ├── docusaurus-plugin-content-docs/
│       │   └── current/
│       │       └── ch01-physical-ai-intro/
│       │           └── index.md   # Translated chapter
│       └── code.json              # UI translations (navbar, footer)
```

**Translation Workflow** (future):
1. Extract translatable strings: `npm run write-translations -- --locale ur`
2. Use LiteLLM/Groq API to translate docs/*.md → i18n/ur/.../*.md
3. Cache translations in Qdrant (per constitution) to avoid redundant API calls
4. Build with locale: `npm run build -- --locale ur`

**Alternatives Considered**:
- **Manual Translation Files**: Harder to maintain, no routing. Docusaurus i18n is standard for docs sites.
- **External Service** (Crowdin): Overkill for 13 chapters (~65k words), adds dependency, $150/month for team plan.
- **Google Translate API**: Not free tier, quality lower than LiteLLM with Groq backend (GPT-4 Turbo).

**References**:
- [Docusaurus i18n Guide](https://docusaurus.io/docs/i18n/introduction)
- [LiteLLM Translation API](https://docs.litellm.ai/docs/translation)
- [Groq Cloud (Fast Inference)](https://groq.com/)

---

## Technology Stack Summary

**Finalized Technologies**:

| Category          | Technology                | Version   | Rationale                                                                 |
|-------------------|---------------------------|-----------|---------------------------------------------------------------------------|
| Runtime           | Node.js                   | 20.x LTS  | Long-term support, stable, required by Docusaurus 3.x                    |
| Framework         | Docusaurus                | 3.5.2+    | Static site generator optimized for docs, MDX support, plugin ecosystem   |
| UI Library        | React                     | 18+       | Required by Docusaurus, component-based architecture for MCQs             |
| Content Format    | MDX                       | 3.x       | Markdown + JSX, allows React components in content                        |
| Diagrams          | Mermaid                   | 10.x      | Declarative diagram syntax, renders to SVG at build time                  |
| Syntax Highlighting | Prism React Renderer    | 2.x       | Code highlighting for Python, Bash, XML, YAML, JSON                       |
| CSS Framework     | Docusaurus Built-in       | N/A       | Infima CSS framework (customizable, responsive, dark mode)                |
| Testing           | Jest + React Testing Lib  | 29.x/14.x | Unit tests for MCQ components, snapshot tests for rendering               |
| CI/CD             | GitHub Actions            | N/A       | Free CI/CD for public repos, caching, Lighthouse CI integration           |
| Deployment        | GitHub Pages (primary)    | N/A       | Free static hosting, CDN, custom domain support                           |
| Deployment (alt)  | Vercel (fallback)         | N/A       | Unlimited bandwidth, preview deploys, Next.js optimized                   |
| i18n              | Docusaurus i18n Plugin    | N/A       | Built-in, RTL support for Urdu, locale routing                            |

**Dependencies** (package.json excerpt):
```json
{
  "dependencies": {
    "@docusaurus/core": "^3.5.2",
    "@docusaurus/preset-classic": "^3.5.2",
    "@docusaurus/theme-mermaid": "^3.5.2",
    "react": "^18.2.0",
    "react-dom": "^18.2.0"
  },
  "devDependencies": {
    "@docusaurus/types": "^3.5.2",
    "@testing-library/react": "^14.0.0",
    "jest": "^29.7.0",
    "@lhci/cli": "^0.13.0"
  }
}
```

**Build Pipeline**:
1. Developer pushes to `main` branch
2. GitHub Actions triggers (workflow: deploy.yml)
3. Checkout code, setup Node.js 20 with npm cache
4. Install dependencies: `npm ci` (uses package-lock.json)
5. Build site: `npm run build` (generates static HTML in build/)
6. Run Lighthouse CI: Check performance (>90), accessibility (>90), best practices
7. Deploy to gh-pages branch: `peaceiris/actions-gh-pages` action
8. GitHub Pages serves site at https://<username>.github.io/physical-ai-textbook/

**Performance Targets** (Lighthouse scores):
- Performance: >90 (TTI <5s on 4G, FCP <2s)
- Accessibility: >90 (WCAG 2.1 AA compliance)
- Best Practices: >90 (HTTPS, no console errors, modern image formats)
- SEO: >90 (meta tags, sitemap, structured data)

---

## Research Complete

**All Technology Decisions Finalized**: ✅

**Key Outcomes**:
1. Docusaurus 3.x chosen for React/MDX integration, plugin ecosystem, performance
2. Client-side MCQs with React hooks (no backend) for instant feedback
3. Mermaid diagrams via built-in plugin (code fence syntax, SVG output)
4. GitHub Actions + GitHub Pages for free CI/CD and hosting (Vercel as fallback)
5. i18n stub (Urdu locale) for future translation without refactoring

**Next Phase**: Phase 1 - Design & Contracts (data models, API contracts, quickstart guide)

**ADR Suggestions**:
- `/sp.adr why-docusaurus-over-gitbook` - Document rationale for framework choice
- `/sp.adr client-side-mcqs-no-backend` - Justify static site approach for assessments
- `/sp.adr github-actions-primary-deployment` - Explain CI/CD pipeline choice
