import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className="relative overflow-hidden bg-gray-900 text-white py-16 md:py-24 lg:py-32">
      {/* Background Glows */}
      <div className="absolute top-0 left-1/2 -translate-x-1/2 w-full h-full max-w-7xl pointer-events-none">
        <div className="absolute top-[-20%] left-[-10%] w-[500px] h-[500px] bg-blue-600/30 rounded-full blur-[120px] mix-blend-screen animate-blob"></div>
        <div className="absolute top-[20%] right-[-10%] w-[400px] h-[400px] bg-purple-600/30 rounded-full blur-[100px] mix-blend-screen animate-blob animation-delay-2000"></div>
        <div className="absolute bottom-[-20%] left-[20%] w-[600px] h-[600px] bg-teal-500/20 rounded-full blur-[120px] mix-blend-screen animate-blob animation-delay-4000"></div>
      </div>

      <div className="container relative z-10 text-center">
        <div className="inline-flex items-center gap-2 px-3 py-1 rounded-full bg-white/10 border border-white/20 backdrop-blur-md mb-8 animate-fade-in-up">
          <span className="flex h-2 w-2 relative">
            <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-green-400 opacity-75"></span>
            <span className="relative inline-flex rounded-full h-2 w-2 bg-green-500"></span>
          </span>
          <span className="text-xs font-medium tracking-wide text-green-300">Hackathon I Edition</span>
        </div>

        <Heading as="h1" className="text-4xl sm:text-5xl md:text-7xl font-extrabold tracking-tight mb-6 animate-fade-in-up animation-delay-100">
          Build the <span className="text-transparent bg-clip-text bg-gradient-to-r from-blue-400 via-teal-400 to-green-400">Body</span> <br className="hidden md:block" />
          for the <span className="text-transparent bg-clip-text bg-gradient-to-r from-purple-400 via-pink-400 to-red-400">Digital Brain</span>
        </Heading>
        
        <p className="text-xl md:text-2xl text-gray-300 max-w-2xl mx-auto mb-10 leading-relaxed animate-fade-in-up animation-delay-200">
          The definitive open-source textbook for <b>Physical AI</b>. 
          Master ROS 2, Isaac Sim, and VLA Models to create autonomous humanoid robots.
        </p>
        
        <div className="flex flex-col sm:flex-row items-center justify-center gap-4 animate-fade-in-up animation-delay-300">
          <Link
            className="button button--primary button--lg px-8 py-4 text-lg font-bold rounded-full shadow-lg shadow-blue-500/30 hover:scale-105 transition-transform"
            to="/docs/module-1-ros2/foundations-physical-ai">
            Start Learning ✨
          </Link>
          <Link
            className="button button--secondary button--lg px-8 py-4 text-lg font-bold rounded-full bg-white/10 border border-white/20 backdrop-blur-md hover:bg-white/20 transition-all"
            to="https://github.com/devhasnainraza/Physical-AI-Humanoid-Robotics-Book">
            View on GitHub
          </Link>
        </div>

        {/* Tech Stack Strip */}
        <div className="mt-16 pt-8 border-t border-white/10 flex flex-wrap justify-center gap-8 opacity-60 grayscale hover:grayscale-0 transition-all duration-500">
            <span className="text-sm font-bold tracking-widest text-gray-500">POWERED BY</span>
            <img src="https://upload.wikimedia.org/wikipedia/commons/b/bb/Ros_logo.svg" className="h-8 w-auto invert" alt="ROS 2" />
            <img src="https://upload.wikimedia.org/wikipedia/sco/thumb/2/21/Nvidia_logo.svg/960px-Nvidia_logo.svg.png?20150924223142" className="h-6 w-auto invert" alt="NVIDIA" />
            <img src="https://upload.wikimedia.org/wikipedia/commons/1/19/Unity_Technologies_logo.svg" className="h-8 w-auto invert" alt="Unity" />
            <img src="https://upload.wikimedia.org/wikipedia/commons/8/8a/Google_Gemini_logo.svg" className="h-6 w-auto invert" alt="Gemini" />
        </div>
      </div>
    </header>
  );
}

const Curriculum = () => (
    <section className="relative py-24 bg-gray-50 dark:bg-black overflow-hidden">
        {/* Decorative Background Grid */}
        <div className="absolute inset-0 bg-grid-slate-100 dark:bg-grid-slate-900/[0.04] bg-[bottom_1px_center] mask-image-linear-to-t"></div>

        <div className="container relative z-10">
            <div className="text-center max-w-3xl mx-auto mb-20">
                <span className="text-sm font-bold tracking-widest text-blue-600 dark:text-blue-400 uppercase bg-blue-100 dark:bg-blue-900/30 px-3 py-1 rounded-full">Roadmap</span>
                <h2 className="text-4xl md:text-5xl font-extrabold mt-4 mb-6 bg-clip-text text-transparent bg-gradient-to-r from-gray-900 via-gray-700 to-gray-900 dark:from-white dark:via-gray-300 dark:to-white">
                    From Zero to Autonomous
                </h2>
                <p className="text-xl text-gray-600 dark:text-gray-400 leading-relaxed">
                    A structured 4-module journey designed to take you from basic Python scripts to deploying Foundation Models on humanoid robots.
                </p>
            </div>
            
            <div className="grid md:grid-cols-2 lg:grid-cols-4 gap-8 relative">
                {/* Connecting Line (Desktop) */}
                <div className="hidden lg:block absolute top-12 left-[10%] right-[10%] h-0.5 bg-gradient-to-r from-blue-500 via-purple-500 to-pink-500 opacity-20 dashed-line"></div>

                {[
                    { title: "The Nervous System", desc: "Master ROS 2 nodes, topics, and real-time control theory.", color: "bg-blue-500", icon: "⚡" },
                    { title: "The Digital Twin", desc: "Build physics-accurate simulations in Gazebo & Unity.", color: "bg-teal-500", icon: "🏗️" },
                    { title: "The AI Brain", desc: "Implement VSLAM, Nav2, and NVIDIA Isaac ROS.", color: "bg-purple-500", icon: "🧠" },
                    { title: "Vision-Language-Action", desc: "Deploy Transformer models for end-to-end autonomy.", color: "bg-pink-500", icon: "🤖" },
                    { title: "Adv. Locomotion", desc: "MPC & Whole-Body Control for stable walking.", color: "bg-orange-500", icon: "fg" },
                    { title: "Dexterous Manipulation", desc: "Grasping, Tactile Sensing & Dual-Arm tasks.", color: "bg-yellow-500", icon: "✋" },
                    { title: "Human-Robot Interaction", desc: "Social Navigation & ISO Safety Standards.", color: "bg-red-500", icon: "🤝" },
                ].map((item, i) => (
                    <div key={i} className="relative group">
                        <div className={`w-24 h-24 mx-auto ${item.color} bg-opacity-10 dark:bg-opacity-20 rounded-3xl flex items-center justify-center mb-6 relative z-10 transition-transform duration-500 group-hover:scale-110 group-hover:rotate-3`}>
                            <div className={`absolute inset-0 ${item.color} opacity-20 blur-xl group-hover:opacity-40 transition-opacity`}></div>
                            <span className="text-4xl filter drop-shadow-md">{item.icon}</span>
                            <div className="absolute -top-2 -right-2 w-8 h-8 bg-white dark:bg-gray-800 rounded-full flex items-center justify-center text-sm font-bold border border-gray-100 dark:border-gray-700 shadow-sm">
                                {i+1}
                            </div>
                        </div>
                        
                        <div className="bg-white dark:bg-gray-900 p-8 rounded-3xl shadow-sm border border-gray-100 dark:border-gray-800 group-hover:shadow-2xl group-hover:border-transparent transition-all duration-300 relative overflow-hidden">
                            <div className={`absolute inset-0 bg-gradient-to-br ${item.color} opacity-0 group-hover:opacity-5 transition-opacity duration-500`}></div>
                            <h3 className="text-xl font-bold mb-3 text-gray-900 dark:text-white group-hover:text-blue-600 dark:group-hover:text-blue-400 transition-colors">{item.title}</h3>
                            <p className="text-gray-500 dark:text-gray-400 text-sm leading-relaxed">{item.desc}</p>
                        </div>
                    </div>
                ))}
            </div>
        </div>
    </section>
);

export default function Home(): React.JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title} | The Future of Robotics`}
      description="The definitive guide to Physical AI and Humanoid Robotics.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <Curriculum />
      </main>
    </Layout>
  );
}