import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';
import 'dotenv/config'; // Load .env file

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Cortex-H1',
  tagline: 'Physical AI Humanoid Robotics',
  favicon: 'img/favicon.ico',

  customFields: {
    geminiApiKey: process.env.GEMINI_API_KEY,
    qdrantUrl: process.env.QDRANT_URL,
    qdrantApiKey: process.env.QDRANT_API_KEY,
    neonConnectionString: process.env.DATABASE_URL,
  },

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Enable Mermaid for Diagrams
  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],

  // Stylesheets for Math (KaTeX) and Fonts
  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.13.24/dist/katex.min.css',
      type: 'text/css',
      integrity:
        'sha384-odtC+0UGzzFL/6PNoE8rX/SPcQDXBJ+uRepguP4QkPCm2LBxH3FA3yUy5gfv/I3X',
      crossorigin: 'anonymous',
    },
    {
      href: 'https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700;800&family=Plus+Jakarta+Sans:wght@400;500;600;700;800&display=swap',
      type: 'text/css',
      rel: 'stylesheet',
    },
  ],

  plugins: [
    'docusaurus-plugin-image-zoom',
  ],

  // Set the production url of your site here
  url: 'https://devhasnainraza.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/Cortex-H1/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'devhasnainraza', // Usually your GitHub org/user name.
  projectName: 'Cortex-H1', // Usually your repo name.
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          remarkPlugins: [remarkMath],
          rehypePlugins: [rehypeKatex],
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    zoom: {
      selector: '.markdown :not(em) > img',
      config: {
        background: {
          light: 'rgb(255, 255, 255)',
          dark: 'rgb(10, 10, 10)'
        }
      }
    },
    navbar: {
      hideOnScroll: true,
      title: 'Cortex-H1',
      logo: {
        alt: 'Cortex-H1 Logo',
        src: 'img/CortexLogo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          href: 'https://github.com/devhasnainraza/Physical-AI-Humanoid-Robotics-Book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
    docs: {
      sidebar: {
        hideable: true,
      },
    },
    footer: {
      style: 'dark',
      logo: {
        alt: 'Cortex-H1 Logo',
        src: 'img/CortexLogo.svg',
        href: '/',
        width: 160,
        height: 51,
      },
      links: [
        {
          title: 'Curriculum',
          items: [
            {
              label: 'Module 1: Foundations',
              to: '/docs/module-1-ros2/foundations-physical-ai',
            },
            {
              label: 'Module 2: Digital Twin',
              to: '/docs/module-2-digital-twin/intro-digital-twin',
            },
            {
              label: 'Module 3: AI Brain',
              to: '/docs/module-3-ai-brain/intro-rl',
            },
            {
              label: 'Module 4: VLA Models',
              to: '/docs/module-4-vla/intro-vla',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub Discussions',
              href: 'https://github.com/devhasnainraza/Physical-AI-Humanoid-Robotics-Book/discussions',
            },
            {
              label: 'Discord Server',
              href: 'https://discord.gg/robotics', // Placeholder or real link if you have one
            },
            {
              label: 'Twitter / X',
              href: 'https://twitter.com/devhasnainraza',
            },
          ],
        },
        {
          title: 'Project',
          items: [
            {
              label: 'About the Author',
              to: 'https://github.com/devhasnainraza',
            },
            {
              label: 'GitHub Repository',
              href: 'https://github.com/devhasnainraza/Physical-AI-Humanoid-Robotics-Book',
            },
            {
              label: 'Report an Issue',
              href: 'https://github.com/devhasnainraza/Physical-AI-Humanoid-Robotics-Book/issues',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Cortex-H1 Project. Built with Docusaurus & Love for Robots.`,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;