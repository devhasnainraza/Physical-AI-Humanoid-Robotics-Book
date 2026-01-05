---
id: chapter-2-robot-hardware
title: 'Chapter 2: Robot Hardware'
sidebar_label: '2. Hardware'
description: 'Deep dive into Actuators, Sensors, Power, and Compute architectures.'
---

# Chapter 2: Robot Hardware

**"Hardware is the stage on which the software dances."**

A robust control algorithm cannot fix a broken gearbox. A precise SLAM system cannot function with a saturated IMU. This chapter explores the physics and engineering of robotic hardware, moving beyond "black boxes" to understand the fundamental limits of machines.

## 2.1 Actuators: The Physics of Motion

The actuator is the transducer that converts electrical energy into mechanical power.

### 2.1.1 BLDC Motors (Brushless Direct Current)
Modern dynamic robots (from drones to humanoids) use BLDC motors almost exclusively.
*   **Construction**:
    *   **Stator**: Fixed outer ring with copper windings (Electromagnets).
    *   **Rotor**: Inner rotating shaft with Permanent Magnets (NdFeB).
*   **Advantages**:
    *   No brushes $\to$ No friction, no sparks, long life.
    *   High power density.
    *   Excellent thermal dissipation (windings are on the outside case).

**Equations of Motion**:
$$ V = IR + L\frac{dI}{dt} + K_e \omega $$
$$ \tau = K_t I $$

*   $V$: Applied Voltage.
*   $K_e$: Back-EMF Constant (Volts / (rad/s)).
*   $K_t$: Torque Constant (Nm / Amp).
*   $\omega$: Angular velocity.

**Key Insight**: At high speeds, the Back-EMF ($K_e \omega$) fights the applied voltage. This limits the maximum current we can inject, and thus the maximum torque. Torque drops as speed increases.

### 2.1.2 Field Oriented Control (FOC)
Driving a BLDC is not simple DC. It requires 3-phase AC waveforms.
*   **Commutation**: We must switch the phases to keep the magnetic field $90^°$ ahead of the rotor.
*   **FOC Algorithm**:
    1.  Read Rotor Angle $\theta$ (Encoder).
    2.  Transform 3-phase currents ($I_a, I_b, I_c$) to 2-axis rotating frame ($I_d, I_q$) using **Clarke & Park Transforms**.
    3.  **$I_q$ (Quadrature)**: Produces Torque. Control this!
    4.  **$I_d$ (Direct)**: Produces Heat (mostly). Set to 0.
    5.  Transform back to drive MOSFETs (Space Vector PWM).

This allows us to control **Torque** directly, making the robot a "Force Source" rather than a "Position Source".

### 2.1.3 Transmission (Gearing)
Motors like to spin fast (3000+ RPM) with low torque. Arms need slow motion (60 RPM) with high torque. We use gears.

**Gear Ratio ($N$)**:
$$ \tau_{out} = N \tau_{in} \cdot \eta $$
$$ \omega_{out} = \frac{\omega_{in}}{N} $$

*   $\eta$: Efficiency (0.7 - 0.95).

**Types**:
1.  **Strain Wave (Harmonic Drive)**: $N=50:1$ to $100:1$. Zero backlash. High friction. Used in Cobots (UR5).
2.  **Cycloidal**: Robust, shock resistant. Used in heavy industrial bots.
3.  **Planetary**: Cheap, efficient. Backlash exists.
4.  **Quasi-Direct Drive (QDD)**: Low ratio ($6:1$ to $10:1$). Used in **Legged Robots** (Unitree, MIT Cheetah).
    *   **Why?** Low reflected inertia ($J_{ref} = N^2 J_{motor}$). The robot feels "transparent" and can handle impacts (jumping) without breaking gears.

## 2.2 Sensors: The Physics of Perception

### 2.2.1 Inertial Measurement Unit (IMU)
The "Inner Ear".
*   **Accelerometer**: Measures proper acceleration (Gravity + Motion). $a_{measured} = a_{motion} - g$.
    *   *Static*: Tells you "Down" vector.
    *   *Dynamic*: Noisy, sensitive to vibration.
*   **Gyroscope**: Measures angular velocity ($\omega$).
    *   *Drift*: Integrating $\omega$ to get angle $\theta$ accumulates error ("Bias drift").

**Sensor Fusion (Complementary Filter)**:
$$ \theta_{est} = \alpha (\theta_{gyro}) + (1-\alpha) (\theta_{accel}) $$
*   Trust Gyro for short term (fast changes).
*   Trust Accel for long term (gravity vector).

### 2.2.2 LiDAR (Light Detection and Ranging)
*   **ToF (Time of Flight)**: Send pulse, measure return time. $d = \frac{c \cdot t}{2}$.
*   **2D LiDAR**: Spinning mirror. Slice of the world. Used for warehouse AMRs.
*   **3D LiDAR**: Multiple beams (16, 32, 64, 128). Used for Autonomous Cars / Humanoids.
*   **Solid State**: No moving parts (MEMS mirrors). Robust.

### 2.2.3 Depth Cameras (RGB-D)
*   **Structured Light**: Projects IR dot pattern. Measures distortion. (Kinect v1). Fails outdoors (Sunlight contains IR).
*   **Active Stereo**: Two cameras + IR Projector (texture). (RealSense D435). Works okay outdoors.
*   **Time of Flight**: Flash LiDAR. (Azure Kinect).

## 2.3 Compute Architectures

### 2.3.1 Microcontrollers (MCU)
The "Spinal Cord".
*   **Examples**: STM32, Teensy 4.1, ESP32.
*   **OS**: RTOS (FreeRTOS) or Bare Metal.
*   **Role**: Real-time Motor Control (10kHz - 40kHz), Safety loops, Sensor polling.
*   **Constraint**: Low RAM (Kb), no Python. C/C++ only.

### 2.3.2 Application Processors (CPU/GPU)
The "Cortex".
*   **Examples**: NVIDIA Jetson Orin, Raspberry Pi 5, Intel NUC.
*   **OS**: Linux (Ubuntu with Preempt-RT patch).
*   **Role**: Path Planning, SLAM, Vision (Neural Networks), High-level Logic.
*   **Constraint**: High power (15W - 60W), Latency jitter.

### 2.3.3 Communication Buses
*   **CAN Bus (Controller Area Network)**: Robust, differential pair. Used for connecting motors.
*   **EtherCAT**: Industrial Ethernet. Extremely fast, synchronized (<1us jitter).
*   **UART / SPI / I2C**: Board-level comms.

## 2.4 Power Systems

### 2.4.1 Batteries
*   **LiPo (Lithium Polymer)**: High "C-Rating" (Discharge rate). Dangerous (Fire risk if punctured/overcharged).
*   **Li-Ion (18650/21700 Cylinders)**: Safer, higher density, lower current. (Tesla packs).

**Voltage Sag**: $V_{term} = V_{oc} - I \cdot R_{internal}$.
Under high load (jumping), voltage drops. Compute needs a **Regulator** (Buck Converter) to stay stable at 5V/12V, or it will brownout and reboot.

## 2.5 Summary Table

| Component | Key Metric | Trade-off |
| :--- | :--- | :--- |
| **Motor** | $K_t$ (Torque Const) | High $K_t$ = High Torque but Low Max Speed. |
| **Gearbox** | Ratio $N$ | High $N$ = High Torque but High Friction/Inertia. |
| **LiDAR** | Points per sec | Resolution vs Update Rate. |
| **Computer** | TOPS / Watt | AI performance vs Battery life. |