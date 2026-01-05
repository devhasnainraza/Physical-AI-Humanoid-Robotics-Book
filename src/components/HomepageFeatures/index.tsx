import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  icon: string;
  description: React.ReactNode;
  color: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Embodied Intelligence',
    icon: '🧠',
    color: 'from-blue-500 to-cyan-500',
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
    color: 'from-purple-500 to-pink-500',
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
    color: 'from-amber-500 to-orange-500',
    description: (
      <>
        Build <b>VLA models</b> that connect voice commands ("Pick up the apple") 
        directly to robotic arm movements using LLMs.
      </>
    ),
  },
];

function Feature({title, icon, color, description}: FeatureItem) {
  return (
    <div className="group relative p-8 h-full bg-white dark:bg-gray-800 rounded-3xl shadow-sm hover:shadow-2xl transition-all duration-500 overflow-hidden border border-gray-100 dark:border-gray-700">
      
      {/* Hover Gradient Background */}
      <div className={`absolute inset-0 bg-gradient-to-br ${color} opacity-0 group-hover:opacity-5 transition-opacity duration-500`}></div>
      
      <div className="text--center mb-6 relative z-10">
        <div className={`w-20 h-20 mx-auto bg-gradient-to-br ${color} rounded-2xl flex items-center justify-center text-4xl shadow-lg transform group-hover:scale-110 group-hover:rotate-3 transition-all duration-500`}>
          {icon}
        </div>
      </div>
      <div className="text--center relative z-10">
        <h3 className="text-2xl font-bold mb-4 bg-clip-text text-transparent bg-gradient-to-r from-gray-900 to-gray-700 dark:from-white dark:to-gray-300">
          {title}
        </h3>
        <p className="text-gray-600 dark:text-gray-400 leading-relaxed">{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): React.JSX.Element {
  return (
    <section className="py-24 bg-white dark:bg-black relative">
      <div className="container relative z-10">
        <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
