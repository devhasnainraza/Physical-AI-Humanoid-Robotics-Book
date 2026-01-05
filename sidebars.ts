import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  textbookSidebar: [
    {
      type: 'category',
      label: 'Textbook',
      items: [
        'textbook/introduction',
        'textbook/chapter-1-foundations',
        'textbook/chapter-2-robot-hardware',
        'textbook/chapter-3-kinematics-dynamics',
        'textbook/chapter-4-control-systems',
        'textbook/chapter-5-perception',
        'textbook/chapter-6-planning',
        'textbook/chapter-7-reinforcement-learning',
        'textbook/chapter-8-slam-nav',
        'textbook/chapter-9-humanoid-robotics',
        'textbook/chapter-10-ethics',
        'textbook/chapter-11-future',
        'textbook/labs',
        'textbook/exercises',
        'textbook/quizzes',
        'textbook/appendix',
        'textbook/slides',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      items: [
        'module-1-ros2/foundations-physical-ai',
        'module-1-ros2/robot-hardware',
        'module-1-ros2/kinematics-dynamics',
        'module-1-ros2/control-systems',
        'module-1-ros2/perception-sensor-fusion',
        'module-1-ros2/planning-decision-making',
        'module-1-ros2/reinforcement-learning-robotics',
        'module-1-ros2/slam-navigation',
        'module-1-ros2/humanoid-robotics',
        'module-1-ros2/safety-ethics-policy',
        'module-1-ros2/future-physical-ai',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      items: [
        'module-2-digital-twin/intro-digital-twin',
        'module-2-digital-twin/urdf-creation',
        'module-2-digital-twin/gazebo-setup',
        'module-2-digital-twin/mujoco-physics',
        'module-2-digital-twin/isaac-lab',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI Brain',
      items: [
        'module-3-ai-brain/intro-rl',
        'module-3-ai-brain/mdp',
        'module-3-ai-brain/q-learning',
        'module-3-ai-brain/policy-gradients',
        'module-3-ai-brain/model-based-rl',
        'module-3-ai-brain/sim-to-real',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA',
      items: [
        'module-4-vla/intro-vla',
        'module-4-vla/foundation-models',
        'module-4-vla/vision-transformers',
        'module-4-vla/multimodal-fusion',
        'module-4-vla/vla-architectures',
        'module-4-vla/future-of-vla',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Advanced Locomotion',
      items: [
        'module-5-locomotion/intro-locomotion',
        'module-5-locomotion/mpc-control',
        'module-5-locomotion/whole-body-control',
        'module-5-locomotion/learning-to-walk',
      ],
    },
    {
      type: 'category',
      label: 'Module 6: Manipulation',
      items: [
        'module-6-manipulation/intro-manipulation',
        'module-6-manipulation/grasping-theory',
        'module-6-manipulation/tactile-sensing',
        'module-6-manipulation/bimanual-manipulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 7: HRI',
      items: [
        'module-7-hri/intro-hri',
        'module-7-hri/social-navigation',
        'module-7-hri/safety-standards',
      ],
    },
  ],
};

export default sidebars;