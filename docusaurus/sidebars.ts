import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Welcome',
    },
    {
      type: 'category',
      label: 'Chapters',
      collapsed: false,
      items: [
        'ch01-physical-ai-intro/ch01',
        'ch02-ros2-fundamentals/ch02',
        'ch03-robot-modeling/ch03',
        'ch04-gazebo-simulation/ch04',
        'ch05-unity-simulation/ch05',
        'ch06-isaac-sim/ch06',
        'ch07-vla-models/ch07',
        'ch08-humanoid-kinematics/ch08',
        'ch09-locomotion/ch09',
        'ch10-manipulation/ch10',
        'ch11-conversational-ai/ch11',
        'ch12-hardware-integration/ch12',
        'ch13-capstone-project/ch13',
      ],
    },
    {
      type: 'doc',
      id: 'hardware-requirements',
      label: 'Hardware Requirements',
    },
  ],
};

export default sidebars;
