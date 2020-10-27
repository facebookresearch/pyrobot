/**
 * Copyright (c) 2017-present, Facebook, Inc.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

// See https://docusaurus.io/docs/site-config for all the possible
// site configuration options.

// List of projects/orgs using your project for the users page.
const users = [
  {
    caption: 'User1',
    // You will need to prepend the image path with your baseUrl
    // if it is not '/', like: '/test-site/img/docusaurus.svg'.
    image: '/img/pyrobot.svg',
    infoLink: 'https://github.com/facebookresearch/pyrobot',
    pinned: true,
  },
];

const siteConfig = {
  title: 'PyRobot', // Title for your website.
  tagline: 'An Open Source Robotics Research Platform',
  // url: 'https://pyrobot-v1-website.netlify.com', // Your website URL
  // baseUrl: '/', // Base URL for your project */
  url: 'https://pyrobot.org', // Your website URL
  // url: 'https://pyrobot-v1-website.netlify.com', // Your website URL
  // baseUrl: '/', // Base URL for your project */
  baseUrl: '/', // Base URL for your project */
  // For github.io type URLs, you would set the url and baseUrl like:
  //   url: 'https://facebook.github.io',
  //   baseUrl: '/test-site/',

  // Used for publishing and more
  // projectName: 'pyrobot',
  cname: 'pyrobot.org',
  projectName: 'pyrobot',
  organizationName: 'facebookresearch',
  // organizationName: 'facebook',
  // For top-level user or org sites, the organization is still the same.
  // e.g., for the https://JoelMarcey.github.io site, it would be set like...
  //   organizationName: 'JoelMarcey'

  // For no header links in the top nav bar -> headerLinks: [],
  headerLinks: [
    {doc: 'overview', label: 'Tutorials'},
    {href: 'https://pyrobot-docs.readthedocs.io/en/latest/#', label: 'API'},
    {doc: 'datasets', label: 'Datasets'},
    {doc: 'faq', label: 'Help'},
    {href: 'https://github.com/facebookresearch/pyrobot', label: 'GitHub'},
    {href: 'https://pyrobot-next.readthedocs.io/en/api_0.4/', label: 'next-API'},
    {href: 'https://github.com/facebookresearch/pyrobot/tree/API_0.4', label: 'next-Github'}
    // {blog: true, label: 'Blog'}
  ],

  // If you have users set above, you add it here:
  users,

  /* path to images for header/footer */
  headerIcon: 'img/pyrobot_icon.svg',
  footerIcon: 'img/pyrobot_icon.svg',
  favicon: 'img/favicon.ico',

  /* Colors for website */
  colors: {
    primaryColor: '#666666',
    secondaryColor: '#999999',
  },

  /* Custom fonts for website */
  /*
  fonts: {
    myFont: [
      "Times New Roman",
      "Serif"
    ],
    myOtherFont: [
      "-apple-system",
      "system-ui"
    ]
  },
  */

  // This copyright info is used in /core/Footer.js and blog RSS/Atom feeds.
  copyright: `Copyright © ${new Date().getFullYear()} Facebook Inc.`,

  highlight: {
    // Highlight.js theme to use for syntax highlighting in code blocks.
    theme: 'default',
  },

  // Add custom scripts here that would be placed in <script> tags.
  scripts: [
  'https://buttons.github.io/buttons.js',
  'https://cdnjs.cloudflare.com/ajax/libs/clipboard.js/2.0.0/clipboard.min.js',
  '/js/code-block-buttons.js',
  ],
  stylesheets: ['/css/code-block-buttons.css'],
  // On page navigation for the current documentation page.
  onPageNav: 'separate',
  // No .html extensions for paths.
  cleanUrl: true,

  // Open Graph and Twitter card images.
  ogImage: 'img/pyrobot_icon.svg',
  twitterImage: 'img/pyrobot_icon.svg',

  // Show documentation's last contributor's name.
  // enableUpdateBy: true,

  // Show documentation's last update time.
  // enableUpdateTime: true,

  // You may provide arbitrary config keys to be used as needed by your
  // template. For example, if you need your repo's URL...
  //   repoUrl: 'https://github.com/facebook/test-site',
};

module.exports = siteConfig;
