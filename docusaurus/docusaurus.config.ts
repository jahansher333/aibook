import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'Learn ROS 2, Isaac Sim, and VLA Models for Humanoid Robots',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  // GitHub Pages config
  url: 'https://jahansher333.github.io',
  baseUrl: '/aibook/',

  organizationName: 'your-username',
  projectName: 'physical-ai-textbook',

  onBrokenLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  themes: ['@docusaurus/theme-mermaid'],
  markdown: {
    mermaid: true,
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/physical-ai-social-card.jpg',

    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },

    
    navbar: {
      title: '🤖 Physical AI',
      hideOnScroll: true,
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },

      items: [
        { type: 'doc', docId: 'intro', position: 'left', label: '🏠 Home' },
        { type: 'docSidebar', sidebarId: 'tutorialSidebar', position: 'left', label: '📚 Chapters' },
        { to: '/docs/hardware-requirements', label: '⚙️ Hardware', position: 'left' },

        {
          type: 'dropdown',
          label: '🚀 Quick Links',
          position: 'left',
          items: [
            { label: '🎯 Get Started', to: '/docs/intro' },
            { label: '🎓 Chapter 1: Physical AI', to: '/docs/ch01-physical-ai-intro/ch01' },
            { label: '🎮 Isaac Sim (Ch 6)', to: '/docs/ch06-isaac-sim/ch06' },
            { label: '🗣️ VLA Models (Ch 7)', to: '/docs/ch07-vla-models/ch07' },
            { label: '🎯 Capstone Project', to: '/docs/ch13-capstone-project/ch13' },
          ],
        },

        { type: 'search', position: 'right' },
        {
          href: 'https://github.com/your-org/physical-ai-textbook',
          label: '⭐ Star on GitHub',
          position: 'right',
        },

        // ✅ Custom Auth Button - temporarily replaced with standard link
        { type: 'dropdown', label: '👤 Account', position: 'right', items: [
          { label: 'Sign In', to: '/login' },
          { label: 'Profile', to: '/profile' },
          { label: 'Sign Out', to: '/logout' }
        ]}

      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: '📚 Quick Start',
          items: [
            { label: 'Get Started', to: '/docs/intro' },
            { label: 'Chapter 1: Physical AI', to: '/docs/ch01-physical-ai-intro/ch01/' },
            { label: 'Chapter 2: ROS 2', to: '/docs/ch02-ros2-fundamentals/ch02/' },
            { label: 'Hardware Guide', to: '/docs/hardware-requirements/' },
          ],
        },
        {
          title: '🎓 Learn',
          items: [
            { label: 'Simulation (Ch 4-6)', to: '/docs/ch04-gazebo-simulation/ch04' },
            { label: 'Advanced AI (Ch 7-9)', to: '/docs/ch07-vla-models/ch07' },
            { label: 'Hardware Deploy (Ch 12)', to: '/docs/ch12-hardware-integration/ch12' },
            { label: 'Capstone Project', to: '/docs/ch13-capstone-project/ch13' },
          ],
        },
        {
          title: '🛠️ Resources',
          items: [
            { label: 'ROS 2 Documentation', href: 'https://docs.ros.org/en/humble/' },
            { label: 'NVIDIA Isaac Sim', href: 'https://docs.omniverse.nvidia.com/isaacsim/latest/index.html' },
            { label: 'OpenAI GPT-4', href: 'https://platform.openai.com/docs' },
            { label: 'Gazebo Tutorials', href: 'https://gazebosim.org/docs' },
          ],
        },
        {
          title: '💬 Community',
          items: [
            { label: 'GitHub Repository', href: 'https://github.com/your-org/physical-ai-textbook' },
            { label: 'Report Issues', href: 'https://github.com/your-org/physical-ai-textbook/issues' },
            { label: 'Discussions', href: 'https://github.com/your-org/physical-ai-textbook/discussions' },
            { label: 'Contributing Guide', href: 'https://github.com/your-org/physical-ai-textbook/blob/main/CONTRIBUTING.md' },
          ],
        },
      ],

      copyright: `
        <div style="margin-top: 2rem; padding-top: 2rem; border-top: 1px solid var(--ifm-color-emphasis-200);">
          <p><strong>Physical AI & Humanoid Robotics Textbook</strong></p>
          <p style="font-size: 0.9rem;">Built with ❤️ using <a href="https://docusaurus.io" target="_blank">Docusaurus</a></p>
          <p style="font-size: 0.85rem;">
            Content: <a href="https://creativecommons.org/licenses/by-sa/4.0/" target="_blank">CC BY-SA 4.0</a> |
            Code: <a href="https://opensource.org/licenses/MIT" target="_blank">MIT License</a>
          </p>
          <p style="font-size: 0.85rem;">© ${new Date().getFullYear()} Physical AI Education Initiative</p>
        </div>
      `,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'json'],
    },

    mermaid: {
      theme: { light: 'default', dark: 'dark' },
      options: {
        maxTextSize: 50000,
        maxEdges: 200,
      },
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
