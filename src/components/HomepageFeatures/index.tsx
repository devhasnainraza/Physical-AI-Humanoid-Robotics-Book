import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  icon: string;
  description: React.ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Embodied Intelligence',
    icon: '🧠',
    description: (
      <>
        Move beyond chatbots. Learn how to deploy AI agents into physical humanoid robots
        using <b>ROS 2</b> and <b>NVIDIA Isaac</b>.
      </>
    ),
  },
  {
    title: 'Sim-to-Real Transfer',
    icon: '🔮',
    description: (
      <>
        Train in the <b>Digital Twin</b> (Gazebo/Isaac Sim) and deploy to the 
        <b>Unitree Go2</b> or <b>Jetson Orin</b> with zero code changes.
      </>
    ),
  },
  {
    title: 'Vision-Language-Action',
    icon: '👁️',
    description: (
      <>
        Build <b>VLA models</b> that connect voice commands ("Pick up the apple") 
        directly to robotic arm movements using LLMs.
      </>
    ),
  },
];

function Feature({title, icon, description}: FeatureItem) {
  return (
    <div className="p-8 h-full bg-white dark:bg-[#151923] rounded-lg border border-slate-200 dark:border-slate-800 hover:border-blue-500 dark:hover:border-blue-500 transition-colors">
      <div className="flex items-center gap-4 mb-4">
        <div className="flex items-center justify-center w-12 h-12 bg-blue-50 dark:bg-blue-900/20 rounded-lg text-2xl">
          {icon}
        </div>
        <h3 className="text-xl font-bold text-slate-900 dark:text-white mb-0">
          {title}
        </h3>
      </div>
      <p className="text-slate-600 dark:text-slate-400 leading-relaxed text-sm md:text-base">
        {description}
      </p>
    </div>
  );
}

export default function HomepageFeatures(): React.JSX.Element {
  return (
    <section className="py-16 bg-white dark:bg-[#0b0e14] border-y border-slate-100 dark:border-slate-800">
      <div className="container px-4">
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6 md:gap-8">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
