import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Embodied Intelligence',
  tagline: 'Physical AI & Humanoid Robotics',
  favicon: 'https://github.com/MuhammedSuhaib.png',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://Panaversity.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/SpecKit-Plus/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Panaversity', // Usually your GitHub org/user name.
  projectName: 'embodied-intelligence-book', // Usually your repo name.
  deploymentBranch: 'gh-pages', // Branch to deploy to

  onBrokenLinks: 'warn',

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
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],
  scripts: [
      {
        src: 'https://cdn.jsdelivr.net/npm/@openai/chatkit/dist/chatkit.umd.js',
        async: true,
      },
    ],
  themeConfig: {
    // Replace with your project's social card
    image: 'img/matrix-bg.png',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Embodied Intelligence',
      logo: {
        alt: 'My Site Logo',
        src: 'img/matrix-bg.png',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          href: 'https://github.com/MuhammedSuhaib/dockathon',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook',
          items: [
            {
              label: 'Robotic Nervous System',
              to: '/docs/module-01-robotic-nervous-system/intro',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'X',
              href: 'https://x.com/cuhaib1',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/MuhammedSuhaib/dockathon',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Panaversity. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
