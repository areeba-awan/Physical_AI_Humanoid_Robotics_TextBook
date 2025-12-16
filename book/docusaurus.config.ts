import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Guide to Embodied AI',
  favicon: 'img/logo.svg',

  url: 'https://hacks022.vercel.app',
  baseUrl: '/',

  organizationName: 'physicalai',
  projectName: 'physicalai-textbook',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        htmlLang: 'en',
        direction: 'ltr',
      },
      ur: {
        label: 'اردو',
        htmlLang: 'ur',
        direction: 'rtl',
      },
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/areeba-awan/Physical-AI-Textbook/tree/main/book/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/social-card.jpg',
    navbar: {
      title: 'Physical AI',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book Overview',
        },
        {
          type: 'dropdown',
          label: 'Chapters',
          position: 'left',
          items: [
            {
              label: 'Chapter 1: ROS 2',
              to: '/docs/module-1-ros2/chapter-1-intro',
            },
            {
              label: 'Chapter 2: Simulation',
              to: '/docs/module-2-simulation/chapter-1-intro',
            },
            {
              label: 'Chapter 3: NVIDIA Isaac',
              to: '/docs/module-3-isaac/chapter-1-intro',
            },
            {
              label: 'Chapter 4: VLA',
              to: '/docs/module-4-vla/chapter-1-intro',
            },
          ],
        },
        {
          href: 'https://github.com/areeba-awan',
          label: 'GitHub',
          position: 'right',
        },
        {
          type: 'localeDropdown',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book',
          items: [
            {
              label: 'Overview',
              to: '/docs/intro',
            },
            {
              label: 'Chapter 1: ROS 2',
              to: '/docs/module-1-ros2/chapter-1-intro',
            },
            {
              label: 'Chapter 2: Simulation',
              to: '/docs/module-2-simulation/chapter-1-intro',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/ros2',
            },
            {
              label: 'Discord',
              href: 'https://discord.gg/ros',
            },
            {
              label: 'X (Twitter)',
              href: 'https://twitter.com/openrobotics',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/areeba-awan',
            },
            {
              label: 'ROS 2 Documentation',
              href: 'https://docs.ros.org/en/humble/',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'cpp'],
    },
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
