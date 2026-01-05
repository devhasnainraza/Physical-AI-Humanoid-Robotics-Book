--- 
id: lab-7-lipm-locomotion
title: 'Lab 7: Humanoid Locomotion with LIPM'
sidebar_label: 'Lab 7: LIPM Locomotion'
---

# Lab 7: Humanoid Locomotion with the Linear Inverted Pendulum Model (LIPM)

## Objective
This lab delves into the core of stable bipedal locomotion for humanoid robots using the **Linear Inverted Pendulum Model (LIPM)**. You will implement a basic LIPM planner in Python to generate stable Center of Mass (CoM) trajectories for a single step, laying the groundwork for more complex walking gaits.

## Theoretical Background
As discussed in **Chapter 9: Humanoid Robotics**, stable bipedal walking is fundamentally about controlling the robot's **Center of Mass (CoM)** relative to its **Zero Moment Point (ZMP)**. The LIPM simplifies the robot's dynamics, allowing us to generate CoM trajectories that ensure the ZMP remains within the support polygon, thus preventing the robot from falling.

*   **LIPM**: A simplified model where the robot's mass is concentrated at a single point (CoM) and its height above the ground (`z_c`) is constant.
*   **ZMP**: The point on the ground where the net moment of all forces is zero. It must stay within the support polygon for stability.
*   **CoM Trajectory**: The path the robot's CoM follows during walking.

The equation of motion for the LIPM in the x-direction is:
$$ c\ddot{x} - \omega^2 x = 0 $$
where $c\omega = c\sqrt{g/z_c}$ is the natural frequency, $g$ is gravity, and $z_c$ is the constant CoM height.
The general solution for the CoM position $x(t)$ and velocity $c\dot{x}(t)$ is:
$$ x(t) = C_1 c\cosh(\omega t) + C_2 c\sinh(\omega t) $$
$$ c\dot{x}(t) = c\omega (C_1 c\sinh(\omega t) + C_2 c\cosh(\omega t)) $$
$C_1$ and $C_2$ are constants determined by initial conditions.

## Prerequisites
*   **Python 3.10+**: With `numpy`.
*   **ROS 2 Humble Development Environment**: (for general setup, though this lab is Python-only).

## Step-by-Step Instructions

### Step 1: Create a New Python Package for Locomotion Planning
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_humanoid_planner
cd my_humanoid_planner
```

### Step 2: Implement the LIPM Planner
Create a file named `~/ros2_ws/src/my_humanoid_planner/lipm_planner.py` and add the following Python code:

```python
import numpy as np

class LIPMPlanner:
    """
    Implements a basic Linear Inverted Pendulum Model (LIPM) planner
    to generate Center of Mass (CoM) trajectories for stable walking.
    """
    def __init__(self, com_height_z, gravity=9.81):
        """
        Initializes the LIPM planner.
        Args:
            com_height_z (float): The constant height of the Center of Mass (z_c) in meters.
            gravity (float): Acceleration due to gravity (g) in m/s^2.
        """
        if com_height_z <= 0:
            raise ValueError("CoM height (z_c) must be positive.")
        self.z_c = com_height_z
        self.g = gravity
        self.omega = np.sqrt(self.g / self.z_c) # Natural frequency of the LIPM

    def calculate_com_trajectory(self, x0, vx0, t_start, t_end, num_points=100):
        """
        Calculates the CoM trajectory (position and velocity) given initial conditions
        and a time interval.
        Args:
            x0 (float): Initial CoM position in x-direction.
            vx0 (float): Initial CoM velocity in x-direction.
            t_start (float): Start time of the trajectory.
            t_end (float): End time of the trajectory.
            num_points (int): Number of points to generate for the trajectory.
        Returns:
            tuple: (times, x_trajectory, vx_trajectory)
        """
        if t_end <= t_start:
            raise ValueError("t_end must be greater than t_start.")

        # Constants C1 and C2 from initial conditions (at t=t_start)
        # x0 = C1 cosh(omega * t_start) + C2 sinh(omega * t_start)
        # vx0 = omega (C1 sinh(omega * t_start) + C2 cosh(omega * t_start))
        
        # Solving for C1 and C2
        cosh_t0 = np.cosh(self.omega * t_start)
        sinh_t0 = np.sinh(self.omega * t_start)
        
        det = self.omega * (cosh_t0**2 - sinh_t0**2) # Should be omega
        
        C1 = (vx0 * cosh_t0 - x0 * self.omega * sinh_t0) / self.omega
        C2 = (x0 * self.omega * cosh_t0 - vx0 * sinh_t0) / self.omega
        
        times = np.linspace(t_start, t_end, num_points)
        x_trajectory = C1 * np.cosh(self.omega * times) + C2 * np.sinh(self.omega * times)
        vx_trajectory = self.omega * (C1 * np.sinh(self.omega * times) + C2 * np.cosh(self.omega * times))

        return times, x_trajectory, vx_trajectory

    def generate_single_step_com(self, initial_com_x, initial_com_vx, step_duration, desired_zmp_x=0.0):
        """
        Generates a CoM trajectory for a single step assuming a constant ZMP.
        Args:
            initial_com_x (float): CoM x-position at the start of the step.
            initial_com_vx (float): CoM x-velocity at the start of the step.
            step_duration (float): Duration of the single step.
            desired_zmp_x (float): Desired ZMP position during this step (relative to the support foot).
        Returns:
            tuple: (times, x_trajectory, vx_trajectory, final_com_x, final_com_vx)
        """
        # Adjust initial CoM relative to desired ZMP
        x0_rel_zmp = initial_com_x - desired_zmp_x
        vx0_rel_zmp = initial_com_vx

        # Calculate C1 and C2 for the relative motion
        cosh_t0 = np.cosh(self.omega * 0) # t_start is 0 for simplicity here
        sinh_t0 = np.sinh(self.omega * 0)
        
        C1 = (vx0_rel_zmp * cosh_t0 - x0_rel_zmp * self.omega * sinh_t0) / self.omega
        C2 = (x0_rel_zmp * self.omega * cosh_t0 - vx0_rel_zmp * sinh_t0) / self.omega
        
        times = np.linspace(0, step_duration, 100)
        x_trajectory_rel_zmp = C1 * np.cosh(self.omega * times) + C2 * np.sinh(self.omega * times)
        vx_trajectory_rel_zmp = self.omega * (C1 * np.sinh(self.omega * times) + C2 * np.cosh(self.omega * times))
        
        # Convert back to absolute CoM position
        x_trajectory = x_trajectory_rel_zmp + desired_zmp_x
        vx_trajectory = vx_trajectory_rel_zmp

        final_com_x = x_trajectory[-1]
        final_com_vx = vx_trajectory[-1]

        return times, x_trajectory, vx_trajectory, final_com_x, final_com_vx

```

### Step 3: Create an Executable Script to Run the Planner
Create a file named `~/ros2_ws/src/my_humanoid_planner/run_lipm_planner.py` and add the following code:

```python
import matplotlib.pyplot as plt
from my_humanoid_planner.lipm_planner import LIPMPlanner

def main():
    # Initialize LIPM Planner (e.g., for a humanoid 0.9m tall)
    planner = LIPMPlanner(com_height_z=0.9)

    # Initial conditions
    initial_com_x = 0.0
    initial_com_vx = 0.0
    step_duration = 0.8 # seconds for one step
    desired_zmp_x = 0.0 # ZMP at the origin of the current support foot

    print(f"Planning CoM trajectory for {step_duration}s step with ZMP at {desired_zmp_x}m")

    times, x_traj, vx_traj, final_x, final_vx = planner.generate_single_step_com(
        initial_com_x, initial_com_vx, step_duration, desired_zmp_x
    )

    print(f"\nFinal CoM position: {final_x:.3f}m")
    print(f"Final CoM velocity: {final_vx:.3f}m/s")

    # Plotting the trajectory
    plt.figure(figsize=(10, 6))
    plt.plot(times, x_traj, label='CoM X Position')
    plt.plot(times, vx_traj, label='CoM X Velocity')
    plt.axhline(y=desired_zmp_x, color='r', linestyle='--', label=f'Desired ZMP X = {desired_zmp_x}m')
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('LIPM CoM Trajectory for a Single Step')
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()
```

### Step 4: Update `setup.py` and `package.xml` for `my_humanoid_planner`
**Update `~/ros2_ws/src/my_humanoid_planner/setup.py`**:
```python
from setuptools import setup

package_name = 'my_humanoid_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_lipm = my_humanoid_planner.run_lipm_planner:main',
        ],
    },
)

```

**Update `~/ros2_ws/src/my_humanoid_planner/package.xml`**:
Add these dependencies:
```xml
  <depend>python3-numpy</depend>
  <depend>python3-matplotlib</depend>
```

### Step 5: Build Your Package and Run the Planner
Navigate back to your workspace root and build your package:
```bash
cd ~/ros2_ws
colcon build --packages-select my_humanoid_planner
```
Source your workspace:
```bash
. install/setup.bash
```
Now, run the LIPM planner:
```bash
ros2 run my_humanoid_planner run_lipm
```
This will display a plot of the CoM position and velocity over time.

## Verification
*   The script runs without errors.
*   A plot window appears, showing the CoM position and velocity trajectories.
*   The CoM position should start at `0.0` and move outwards (or inwards, depending on the initial conditions and ZMP).

## Challenge Questions
1.  **Varying ZMP**: Modify `generate_single_step_com` to make `desired_zmp_x` a function of time, mimicking a ZMP that moves within the support foot. How does this affect the CoM trajectory?
2.  **Double Support Phase**: Extend the planner to include a "double support phase" where both feet are on the ground. How would the ZMP dynamics change, and how would you transition between single and double support?
3.  **Ankle Strategy**: In real humanoids, the ankle can apply a small torque to shift the ZMP. How could you incorporate this "ankle strategy" into the LIPM model to achieve a more robust balance?
4.  **3D LIPM**: Extend the model to 3D, controlling both x and y components of the CoM.
