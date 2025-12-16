import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'doc',
      id: 'quickstart',
      label: 'Quickstart Guide',
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 - The Robotic Nervous System',
      collapsed: true,
      collapsible: true,
      items: [
        {
          type: 'doc',
          id: 'module-1-ros2/chapter-1-intro',
          label: 'Module 1.1 Introduction to ROS 2',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/chapter-2-nodes-topics',
          label: 'Module 1.2 Nodes & Topics',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/chapter-3-actions-params',
          label: 'Module 1.3 Actions & Parameters',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/chapter-4-launch-files',
          label: 'Module 1.4 Launch Files',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/chapter-5-custom-packages',
          label: 'Module 1.5 Custom Packages',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/chapter-6-lab',
          label: 'Module 1.6 Hands-on Lab',
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin - Simulation',
      collapsed: true,
      collapsible: true,
      items: [
        {
          type: 'doc',
          id: 'module-2-simulation/chapter-1-intro',
          label: 'Module 2.1 Introduction to Simulation',
        },
        {
          type: 'doc',
          id: 'module-2-simulation/chapter-2-gazebo',
          label: 'Module 2.2 Gazebo Basics',
        },
        {
          type: 'doc',
          id: 'module-2-simulation/chapter-3-urdf',
          label: 'Module 2.3 URDF Robot Models',
        },
        {
          type: 'doc',
          id: 'module-2-simulation/chapter-4-unity',
          label: 'Module 2.4 Unity Integration',
        },
        {
          type: 'doc',
          id: 'module-2-simulation/chapter-5-sensors',
          label: 'Module 2.5 Sensors & Physics',
        },
        {
          type: 'doc',
          id: 'module-2-simulation/chapter-6-lab',
          label: 'Module 2.6 Hands-on Lab',
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac - AI-Robot Brain',
      collapsed: true,
      collapsible: true,
      items: [
        {
          type: 'doc',
          id: 'module-3-isaac/chapter-1-intro',
          label: 'Module 3.1 Introduction to Isaac',
        },
        {
          type: 'doc',
          id: 'module-3-isaac/chapter-2-sim',
          label: 'Module 3.2 Isaac Sim',
        },
        {
          type: 'doc',
          id: 'module-3-isaac/chapter-3-perception',
          label: 'Module 3.3 Perception',
        },
        {
          type: 'doc',
          id: 'module-3-isaac/chapter-4-manipulation',
          label: 'Module 3.4 Manipulation',
        },
        {
          type: 'doc',
          id: 'module-3-isaac/chapter-5-ros-integration',
          label: 'Module 3.5 ROS Integration',
        },
        {
          type: 'doc',
          id: 'module-3-isaac/chapter-6-lab',
          label: 'Module 3.6 Hands-on Lab',
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA - Vision-Language-Action',
      collapsed: true,
      collapsible: true,
      items: [
        {
          type: 'doc',
          id: 'module-4-vla/chapter-1-intro',
          label: 'Module 4.1 Introduction to VLA',
        },
        {
          type: 'doc',
          id: 'module-4-vla/chapter-2-vision',
          label: 'Module 4.2 Vision Systems',
        },
        {
          type: 'doc',
          id: 'module-4-vla/chapter-3-language',
          label: 'Module 4.3 Language Models',
        },
        {
          type: 'doc',
          id: 'module-4-vla/chapter-4-action',
          label: 'Module 4.4 Action Planning',
        },
        {
          type: 'doc',
          id: 'module-4-vla/chapter-5-systems',
          label: 'Module 4.5 Integrated Systems',
        },
        {
          type: 'doc',
          id: 'module-4-vla/chapter-6-capstone',
          label: 'Module 4.6 Capstone Project',
        },
      ],
    },
  ],
};

export default sidebars;
