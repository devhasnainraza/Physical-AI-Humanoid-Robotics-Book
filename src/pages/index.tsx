import React, { useEffect, useRef } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import clsx from 'clsx';

// --- Animated Background ---
const CyberBackground = () => (
  <div className="absolute inset-0 overflow-hidden pointer-events-none">
    <div className="absolute inset-0 bg-noise opacity-30 mix-blend-overlay"></div>
    <div className="absolute inset-0 perspective-grid top-[20%]"></div>
    <div className="absolute top-[-10%] left-[20%] w-[800px] h-[800px] bg-emerald-500/10 rounded-full blur-[120px] mix-blend-screen animate-blob"></div>
    <div className="absolute bottom-[-10%] right-[10%] w-[600px] h-[600px] bg-cyan-500/10 rounded-full blur-[100px] mix-blend-screen animate-blob animation-delay-2000"></div>
  </div>
);

// --- Hero ---
function Hero() {
  return (
    <header className="relative min-h-[95vh] flex items-center justify-center overflow-hidden bg-white dark:bg-[#020617]">
      <CyberBackground />
      <div className="container relative z-10 flex flex-col items-center text-center px-4 pt-20">
        <div className="group relative inline-flex items-center gap-2 px-6 py-2 rounded-full bg-white/40 dark:bg-emerald-900/10 border border-emerald-500/20 backdrop-blur-xl mb-12 shadow-2xl shadow-emerald-500/10 hover:scale-105 transition-transform cursor-default">
          <span className="relative flex h-3 w-3">
            <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-emerald-400 opacity-75"></span>
            <span className="relative inline-flex rounded-full h-3 w-3 bg-emerald-500"></span>
          </span>
          <span className="text-sm font-bold tracking-widest text-emerald-800 dark:text-emerald-300 uppercase">
            V1.0 Public Release
          </span>
          <div className="absolute inset-0 rounded-full bg-gradient-to-r from-emerald-500/0 via-emerald-500/10 to-emerald-500/0 opacity-0 group-hover:opacity-100 transition-opacity animate-shimmer"></div>
        </div>

        <Heading as="h1" className="text-7xl sm:text-8xl md:text-9xl lg:text-[10rem] font-black tracking-tighter text-slate-900 dark:text-white mb-6 leading-[0.9]">
          Physical <br/>
          <span className="text-shimmer">Intelligence</span>
        </Heading>
        
        <p className="text-2xl md:text-3xl text-slate-600 dark:text-slate-400 max-w-4xl mx-auto mb-14 leading-relaxed font-medium tracking-tight">
          The operating system for the next generation of humanoid robots.
          <span className="block mt-4 text-lg md:text-xl text-slate-500 dark:text-slate-500 font-normal">
            From <strong className="text-slate-900 dark:text-white">Real-Time Kernels</strong> to <strong className="text-slate-900 dark:text-white">Foundation Models</strong>.
          </span>
        </p>

        <div className="grid grid-cols-1 sm:grid-cols-2 gap-6 w-full max-w-lg">
          <Link
            className="group relative flex items-center justify-center gap-3 px-8 py-5 text-lg font-bold text-white rounded-2xl bg-slate-900 dark:bg-emerald-600 hover:bg-slate-800 dark:hover:bg-emerald-500 shadow-2xl shadow-emerald-500/20 transition-all overflow-hidden no-underline"
            to="/docs/module-1-ros2/foundations-physical-ai">
            <span className="relative z-10">Start Engineering</span>
            <svg className="w-5 h-5 relative z-10 transition-transform group-hover:translate-x-1" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path strokeLinecap="round" strokeLinejoin="round" strokeWidth="2" d="M13 7l5 5m0 0l-5 5m5-5H6"></path></svg>
            <div className="absolute inset-0 bg-gradient-to-r from-transparent via-white/10 to-transparent translate-x-[-100%] group-hover:translate-x-[100%] transition-transform duration-1000"></div>
          </Link>
          <Link
            className="group flex items-center justify-center gap-3 px-8 py-5 text-lg font-bold text-slate-900 dark:text-white rounded-2xl bg-white/50 dark:bg-white/5 border border-slate-200 dark:border-white/10 backdrop-blur-md hover:bg-white/80 dark:hover:bg-white/10 transition-all no-underline"
            to="https://github.com/devhasnainraza/Physical-AI-Humanoid-Robotics-Book">
            <span>Source Code</span>
          </Link>
        </div>
      </div>
    </header>
  );
}

// --- NEW: The Pipeline Visualization ---
const ThePipeline = () => (
    <section className="py-24 bg-slate-50 dark:bg-[#050a14] border-y border-slate-200 dark:border-slate-800 overflow-hidden">
        <div className="container px-4 text-center">
            <p className="text-sm font-bold tracking-[0.2em] text-emerald-600 dark:text-emerald-400 uppercase mb-16">The Autonomy Pipeline</p>
            
            <div className="relative flex flex-col md:flex-row items-center justify-center gap-8 md:gap-12 max-w-6xl mx-auto">
                {/* Connecting Line (Desktop) */}
                <div className="hidden md:block absolute top-1/2 left-0 w-full h-[2px] bg-slate-200 dark:bg-slate-800 -z-10">
                    <div className="absolute inset-0 bg-emerald-500 w-1/3 animate-flow opacity-50"></div>
                </div>

                {[
                    { title: "Perception", icon: "👁️", desc: "RGB-D + LiDAR", color: "border-blue-500" },
                    { title: "World Model", icon: "🗺️", desc: "VSLAM & Semantics", color: "border-purple-500" },
                    { title: "Reasoning", icon: "🧠", desc: "VLA Transformer", color: "border-emerald-500" },
                    { title: "Planning", icon: "🎯", desc: "Nav2 & MoveIt", color: "border-orange-500" },
                    { title: "Control", icon: "⚡", desc: "DDS Real-Time", color: "border-red-500" },
                ].map((step, i) => (
                    <div key={i} className="relative group">
                        <div className={`w-24 h-24 rounded-2xl bg-white dark:bg-[#0b101e] border-2 ${step.color} shadow-xl flex items-center justify-center text-4xl mb-6 relative z-10 transition-transform group-hover:scale-110`}>
                            {step.icon}
                            <div className="absolute -inset-2 rounded-3xl bg-inherit -z-10 opacity-0 group-hover:opacity-20 transition-opacity blur-lg ring-pulse"></div>
                        </div>
                        <h3 className="text-lg font-bold text-slate-900 dark:text-white">{step.title}</h3>
                        <p className="text-sm text-slate-500 dark:text-slate-400 mt-1">{step.desc}</p>
                    </div>
                ))}
            </div>
        </div>
    </section>
);

// --- NEW: Stats Strip ---
const StatsStrip = () => (
    <div className="grid grid-cols-2 md:grid-cols-4 border-b border-slate-200 dark:border-slate-800">
        {[
            { label: "Engineering Modules", val: "7+" },
            { label: "Hardware Platforms", val: "3" },
            { label: "Community Builders", val: "1.2k" },
            { label: "Open Source", val: "100%" },
        ].map((stat, i) => (
            <div key={i} className="py-12 text-center bg-white dark:bg-[#020617] border-r border-slate-200 dark:border-slate-800 last:border-r-0 hover:bg-slate-50 dark:hover:bg-[#0b101e] transition-colors">
                <div className="text-3xl md:text-4xl font-black text-slate-900 dark:text-white mb-2">{stat.val}</div>
                <div className="text-xs font-bold tracking-widest text-slate-500 uppercase">{stat.label}</div>
            </div>
        ))}
    </div>
);

// --- Bento Grid ---
const MasterCurriculum = () => (
    <section className="py-32 bg-white dark:bg-[#020617] relative">
        <div className="container px-4">
            <div className="text-center mb-20">
                <h2 className="text-5xl md:text-7xl font-black text-slate-900 dark:text-white mb-6">Full-Stack Robotics</h2>
                <p className="text-xl text-slate-600 dark:text-slate-400 max-w-2xl mx-auto">
                    Master the architecture.
                </p>
            </div>

            <div className="grid grid-cols-1 md:grid-cols-6 lg:grid-cols-12 gap-6 auto-rows-[minmax(180px,auto)] max-w-7xl mx-auto">
                {/* 1. The Brain */}
                <div className="md:col-span-6 lg:col-span-8 md:row-span-2 card-holographic rounded-[2.5rem] p-10 flex flex-col justify-between group overflow-hidden">
                    <div className="relative z-10 max-w-xl">
                        <div className="w-20 h-20 bg-gradient-to-br from-emerald-400 to-cyan-500 rounded-3xl flex items-center justify-center text-4xl mb-8 shadow-lg shadow-emerald-500/30 text-white">🧠</div>
                        <h3 className="text-4xl font-bold mb-4 text-slate-900 dark:text-white">The AI Brain</h3>
                        <p className="text-xl text-slate-700 dark:text-slate-300 leading-relaxed mb-8">
                            Implement <strong className="text-emerald-600 dark:text-emerald-400">Visual SLAM</strong>, <strong className="text-emerald-600 dark:text-emerald-400">Nav2</strong>, and <strong className="text-emerald-600 dark:text-emerald-400">Foundation Models</strong>. 
                            Process RGB-D data on the Edge.
                        </p>
                        <div className="flex gap-3 flex-wrap">
                            {['RT-2', 'OpenVLA', 'GPT-4o', 'YOLOv8'].map(tag => (
                                <span key={tag} className="px-3 py-1 rounded-md bg-slate-100 dark:bg-white/10 border border-slate-200 dark:border-white/10 text-sm font-mono font-bold text-slate-600 dark:text-slate-300">{tag}</span>
                            ))}
                        </div>
                    </div>
                    <div className="absolute -right-20 -bottom-20 w-[400px] h-[400px] bg-gradient-to-tl from-emerald-500/20 to-transparent rounded-full blur-[80px] group-hover:scale-125 transition-transform duration-1000"></div>
                </div>

                {/* 2. Simulation */}
                <div className="md:col-span-3 lg:col-span-4 md:row-span-2 card-holographic rounded-[2.5rem] p-10 group relative overflow-hidden">
                    <div className="absolute inset-0 bg-[url('https://upload.wikimedia.org/wikipedia/commons/e/e5/NASA_Mars_Rover.jpg')] bg-cover bg-center opacity-5 group-hover:opacity-10 transition-opacity grayscale mix-blend-overlay"></div>
                    <div className="relative z-10 h-full flex flex-col">
                        <div className="text-5xl mb-auto">🏗️</div>
                        <div>
                            <h3 className="text-2xl font-bold mb-2 text-slate-900 dark:text-white">Digital Twin</h3>
                            <p className="text-slate-600 dark:text-slate-400 mb-6">Zero-cost training in NVIDIA Isaac Sim.</p>
                            <Link className="no-underline"
            to="/docs/module-2-digital-twin/intro-digital-twin">
                                      <span className="text-emerald-600 dark:text-emerald-400 font-bold flex items-center gap-2">Learn Simulation <span className="group-hover:translate-x-1 transition-transform">→</span></span>

          </Link>
                        </div>
                    </div>
                </div>

                {/* 3. Hardware */}
                <div className="md:col-span-3 lg:col-span-5 card-holographic rounded-[2.5rem] p-8 flex flex-col justify-end bg-slate-900 group overflow-hidden border-none relative">
                    <div className="absolute inset-0 bg-[url('https://www.transparenttextures.com/patterns/carbon-fibre.png')] opacity-30"></div>
                    <div className="relative z-10">
                        <span className="font-mono text-emerald-400 text-xs tracking-widest mb-2 block">HARDWARE_INTERFACE</span>
                        <h3 className="text-2xl font-bold text-white mb-1">Unitree Go2</h3>
                        <p className="text-slate-400 text-sm">Low-level motor control via DDS.</p>
                    </div>
                </div>

                {/* 4. Locomotion */}
                <div className="md:col-span-3 lg:col-span-7 card-holographic rounded-[2.5rem] p-8 flex items-center gap-6 group">
                     <div className="w-16 h-16 rounded-full bg-teal-50 dark:bg-teal-500/20 flex items-center justify-center text-3xl">🏃</div>
                     <div>
                        <h3 className="text-2xl font-bold mb-1 text-slate-900 dark:text-white">Locomotion</h3>
                        <p className="text-slate-600 dark:text-slate-400">Model Predictive Control (MPC) & WBC.</p>
                     </div>
                </div>
            </div>
        </div>
    </section>
);

// --- Technical Deep Dive ---
const TechnicalDeepDive = () => (
    <section className="py-32 bg-slate-900 text-white relative overflow-hidden">
         <div className="absolute inset-0 bg-[url('/img/grid-pattern.svg')] opacity-5"></div>
         <div className="container relative z-10 px-4 grid lg:grid-cols-2 gap-20 items-center">
             <div>
                 <div className="inline-block px-3 py-1 rounded bg-emerald-500/10 text-emerald-400 font-mono text-xs mb-6">DEPLOYMENT_TARGET</div>
                 <h2 className="text-4xl md:text-6xl font-black mb-10">Hardware Stack</h2>
                 <div className="space-y-8">
                     {[{ label: "Compute", val: "NVIDIA Jetson AGX Orin 64GB", icon: "💻" }, { label: "Vision", val: "Intel RealSense D435i + LiDAR", icon: "👁️" }, { label: "Actuation", val: "Unitree High-Torque Motors", icon: "⚙️" }, { label: "Kernel", val: "Ubuntu 22.04 Real-Time (PREEMPT_RT)", icon: "🐧" }].map((spec, i) => (
                         <div key={i} className="flex items-center gap-6 group">
                             <div className="w-12 h-12 rounded-xl bg-slate-800 flex items-center justify-center text-2xl group-hover:bg-emerald-600 transition-colors">{spec.icon}</div>
                             <div>
                                 <div className="text-slate-400 text-sm font-bold tracking-wider uppercase">{spec.label}</div>
                                 <div className="text-xl md:text-2xl font-bold font-mono text-white">{spec.val}</div>
                             </div>
                         </div>
                     ))}
                 </div>
             </div>
             <div className="relative perspective-1000">
                 <div className="absolute inset-0 bg-emerald-500/20 blur-[80px] rounded-full"></div>
                 <div className="relative bg-[#0d1117] rounded-xl border border-slate-700 shadow-2xl p-6 font-mono text-sm leading-relaxed overflow-hidden transform rotate-y-[-5deg] rotate-x-[5deg] hover:rotate-0 transition-transform duration-500">
                     <div className="flex gap-2 mb-4 border-b border-slate-800 pb-4">
                         <div className="w-3 h-3 rounded-full bg-red-500"></div><div className="w-3 h-3 rounded-full bg-yellow-500"></div><div className="w-3 h-3 rounded-full bg-green-500"></div>
                     </div>
                     <div className="space-y-2">
                         <p className="text-emerald-400">$ ros2 launch cortex_bringup robot.launch.py</p>
                         <p className="text-slate-400">[INFO] [launch]: All log files can be found below /home/cortex/.ros/log</p>
                         <p className="text-slate-300">[INFO] [hardware_interface]: <span className="text-blue-400">EtherCAT Master connected.</span></p>
                         <p className="text-slate-300">[INFO] [controller_manager]: Loading controller 'joint_trajectory_controller'</p>
                         <p className="text-slate-300">[INFO] [moveit_move_group]: <span className="text-green-400">Ready to take commands.</span></p>
                         <p className="text-emerald-400 mt-4">$ python3 run_inference.py --model rt-2-x</p>
                         <p className="text-slate-300">Loading weights... [100%]</p>
                         <p className="text-yellow-400">>> "Pick up the blue screwdriver"</p>
                         <p className="text-slate-300">Generating trajectory... <span className="animate-pulse">_</span></p>
                     </div>
                 </div>
             </div>
         </div>
    </section>
);

// --- Ecosystem Strip ---
const Ecosystem = () => (
    <div className="py-12 bg-white dark:bg-[#020617] border-t border-slate-200 dark:border-slate-800 overflow-hidden">
        <div className="container px-4 text-center">
            <p className="text-sm font-bold tracking-widest text-slate-400 uppercase mb-8 opacity-60">Powering the Next Generation of Robotics</p>
            <div className="flex flex-wrap justify-center items-center gap-12 md:gap-20 opacity-50 grayscale hover:grayscale-0 transition-all duration-700">
                <h3 className="text-2xl font-black text-slate-900 dark:text-white">ROS 2</h3>
                <h3 className="text-2xl font-black text-slate-900 dark:text-white">NVIDIA</h3>
                <h3 className="text-2xl font-black text-slate-900 dark:text-white">OpenCV</h3>
                <h3 className="text-2xl font-black text-slate-900 dark:text-white">PyTorch</h3>
                <h3 className="text-2xl font-black text-slate-900 dark:text-white">MoveIt</h3>
            </div>
        </div>
    </div>
);

export default function Home(): React.JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title} | The Physical AI Handbook`}
      description="Build autonomous humanoid robots with ROS 2 and VLA Models.">
      <main className="bg-white dark:bg-[#020617]">
        <Hero />
        <StatsStrip />
        <ThePipeline />
        <MasterCurriculum />
        <TechnicalDeepDive />
        <Ecosystem />
        
        {/* Footer CTA */}
        <section className="py-40 bg-white dark:bg-[#020617] relative overflow-hidden text-center">
            <div className="absolute inset-0 bg-grid-emerald opacity-10 pointer-events-none"></div>
            <div className="container relative z-10">
                <h2 className="text-6xl md:text-8xl font-black text-slate-900 dark:text-white mb-10 tracking-tighter">
                    Build the <br/> <span className="text-transparent bg-clip-text bg-gradient-to-r from-emerald-500 to-cyan-500">Future.</span>
                </h2>
                <Link
                    className="inline-flex items-center justify-center px-12 py-6 text-2xl font-bold text-white rounded-full bg-slate-900 dark:bg-emerald-600 hover:scale-105 transition-transform shadow-2xl shadow-emerald-500/20 no-underline"
                    to="/docs/module-1-ros2/foundations-physical-ai">
                    Start Your Journey
                </Link>
            </div>
        </section>
      </main>
    </Layout>
  );
}