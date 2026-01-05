--- 
id: lab-5-rl-locomotion
title: 'Lab 5: Deep Reinforcement Learning for Locomotion'
sidebar_label: 'Lab 5: RL Locomotion'
---

# Lab 5: Deep Reinforcement Learning for Locomotion

## Objective
This lab introduces you to the exciting world of Deep Reinforcement Learning (DRL) for robot locomotion. You will set up a simplified simulated environment using `Gymnasium` and train a basic legged robot to "walk" (move forward) using the **Proximal Policy Optimization (PPO)** algorithm. This lab provides a conceptual foundation for applying DRL to more complex robots in environments like NVIDIA Isaac Lab.

## Theoretical Background
**Reinforcement Learning (RL)**, as discussed in **Chapter 7**, is about an agent learning to make sequential decisions by trial and error to maximize a cumulative reward.
*   **Agent**: Our simulated legged robot.
*   **Environment**: The `Gymnasium` simulation providing states and rewards.
*   **State (Observation)**: Information about the robot (joint angles, velocities, IMU).
*   **Action**: Commands sent to the robot's joints (e.g., torques or target positions).
*   **Reward Function**: Defines what constitutes "good" behavior (e.g., moving forward, not falling).
*   **PPO Algorithm**: A policy optimization algorithm robustly used for continuous control tasks in robotics.

## Prerequisites
*   **ROS 2 Humble Development Environment**: Set up as in Lab 1 (though this lab is mostly Python-centric, the environment is suitable).
*   **Python 3.10+**: With `pip`.
*   **Install necessary libraries**:
    ```bash
    pip install gymnasium stable-baselines3[extra]
    ```

## Step-by-Step Instructions

### Step 1: Create a Custom Legged Robot Environment in Gymnasium
We will create a very simplified 2-DOF legged robot (like a single leg from a humanoid).

1.  Create a new Python file named `~/ros2_ws/src/my_rl_robot/my_legged_env.py` (you might need to create the `my_rl_robot` package first with `ros2 pkg create --build-type ament_python my_rl_robot`).

```python
import gymnasium as gym
from gymnasium import spaces
import numpy as np

class SimpleLeggedRobotEnv(gym.Env):
    """
    A simplified Gymnasium environment for a 2-DOF legged robot.
    The robot's goal is to move its end-effector to a target position.
    """
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}

    def __init__(self, render_mode=None):
        super().__init__()
        
        # Action space: 2 joint position commands (e.g., hip and knee angle targets)
        # Assuming joint angles are normalized between -1 and 1
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        # Observation space: 2 joint angles, 2 joint velocities, 2 end-effector (x,y)
        # Simplified: just joint positions and target
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)

        # --- Robot State ---
        self.joint_angles = np.array([0.0, 0.0], dtype=np.float32) # [hip, knee]
        self.end_effector_pos = np.array([0.0, -0.5], dtype=np.float32) # Initial (x,y)
        self.target_pos = np.array([0.5, -0.5], dtype=np.float32) # Go forward 0.5m

        # --- Simulation Parameters ---
        self.link_length = 0.5 # Both links are 0.5m long
        self.dt = 0.05 # Time step

        self.render_mode = render_mode
        # self.window = None
        # self.clock = None

    def _get_obs(self):
        # Observation = [current_hip_angle, current_knee_angle, target_x, target_y]
        return np.concatenate((self.joint_angles, self.target_pos))

    def _get_info(self):
        return {"distance_to_target": np.linalg.norm(self.end_effector_pos - self.target_pos)}

    def _forward_kinematics(self, hip_angle, knee_angle):
        """Calculates the end-effector (x,y) position from joint angles."""
        x = self.link_length * np.sin(hip_angle) + self.link_length * np.sin(hip_angle + knee_angle)
        y = -self.link_length * np.cos(hip_angle) - self.link_length * np.cos(hip_angle + knee_angle)
        return np.array([x, y])

    def step(self, action):
        # Convert action (normalized -1 to 1) to actual joint angle changes
        # Simple PID-like action: target joint angles are adjusted by action
        self.joint_angles += action * 0.1 # Small step towards target angle
        self.joint_angles = np.clip(self.joint_angles, -np.pi/2, np.pi/2) # Joint limits -90 to 90 deg

        # Update end-effector position
        self.end_effector_pos = self._forward_kinematics(self.joint_angles[0], self.joint_angles[1])

        # --- Reward Calculation ---
        distance_to_target = np.linalg.norm(self.end_effector_pos - self.target_pos)
        
        # Reward for being close to target (negative distance)
        reward = -distance_to_target 

        # Additional small reward for moving forward (increasing x)
        reward += self.end_effector_pos[0] * 0.1
        
        # Penalty for excessive joint movement (energy)
        reward -= np.sum(np.abs(action)) * 0.01

        # --- Termination Condition ---
        terminated = distance_to_target < 0.1 # Reached target
        truncated = False # No time limit in this simple env

        observation = self._get_obs()
        info = self._get_info()

        return observation, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # Reset robot state
        self.joint_angles = np.array([0.0, 0.0], dtype=np.float32)
        self.end_effector_pos = self._forward_kinematics(self.joint_angles[0], self.joint_angles[1])
        # Randomize target slightly for generalization
        self.target_pos = np.array([0.5 + np.random.uniform(-0.1, 0.1), -0.5], dtype=np.float32) 

        observation = self._get_obs()
        info = self._get_info()
        return observation, info

    def render(self):
        # Basic rendering logic (can be expanded with Pygame/Matplotlib)
        pass # Not implementing visual render for this lab

    def close(self):
        pass

```

### Step 2: Write a Training Script Using Stable Baselines3 (PPO)
Create a new Python file named `~/ros2_ws/src/my_rl_robot/train_legged_robot.py`.

```python
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnNoModelImprovement
from my_legged_env import SimpleLeggedRobotEnv # Import our custom environment

if __name__ == "__main__":
    # 1. Create the environment
    # For more complex robotics, this would be an Isaac Lab or Gazebo env
    env = make_vec_env(SimpleLeggedRobotEnv, n_envs=4, seed=0) 

    # 2. Define the PPO agent
    model = PPO(
        "MlpPolicy", # Multi-layer Perceptron Policy
        env,
        verbose=1,
        n_steps=2048, # Number of steps to run for each environment per update
        batch_size=64, # Minibatch size
        gamma=0.99, # Discount factor
        gae_lambda=0.95, # Factor for Generalized Advantage Estimation
        clip_range=0.2, # Clipping parameter for PPO
        ent_coef=0.01, # Entropy coefficient for exploration
        learning_rate=0.0003,
        tensorboard_log="./ppo_legged_robot_log/" # For visualizing training progress
    )

    # 3. Define Callbacks for evaluation and early stopping
    # Create a separate evaluation env
    eval_env = SimpleLeggedRobotEnv()
    stop_train_callback = StopTrainingOnNoModelImprovement(max_no_improve_evals=3, min_evals=5, verbose=1)
    eval_callback = EvalCallback(
        eval_env, 
        callback_on_new_best=stop_train_callback, 
        best_model_save_path="./best_model_legged/",
        log_path="./ppo_legged_robot_log/", 
        eval_freq=1000, 
        deterministic=True, 
        render=False
    )

    # 4. Train the agent
    print("Starting training for Simple Legged Robot...")
    model.learn(total_timesteps=50000, callback=eval_callback) # Train for 50,000 steps
    print("Training finished.")

    # 5. Save the final trained model
    model.save("simple_legged_robot_policy")

    # 6. (Optional) Test the trained agent
    print("\nTesting the trained agent...")
    obs, info = eval_env.reset()
    for i in range(100):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = eval_env.step(action)
        # eval_env.render() # Uncomment if you implement rendering
        if terminated or truncated:
            obs, info = eval_env.reset()
            print("Episode finished, resetting environment.")
```

### Step 3: Update `setup.py` and `package.xml` for `my_rl_robot`
**Update `~/ros2_ws/src/my_rl_robot/setup.py`**:
```python
from setuptools import setup

package_name = 'my_rl_robot'

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
            'train_legged_robot = my_rl_robot.train_legged_robot:main',
        ],
    },
)

```

**Update `~/ros2_ws/src/my_rl_robot/package.xml`**:
Add these dependencies:
```xml
  <depend>python3-gymnasium</depend>
  <depend>python3-stable-baselines3</depend>
  <depend>python3-numpy</depend>
```

### Step 4: Build Your Package and Run Training
Navigate back to your workspace root and build your package:
```bash
cd ~/ros2_ws
colcon build --packages-select my_rl_robot
```
Source your workspace:
```bash
. install/setup.bash
```
Now, run the training script:
```bash
ros2 run my_rl_robot train_legged_robot
```
Observe the training process in your terminal. You will see PPO reporting `rollout/ep_len_mean`, `rollout/ep_rew_mean` (episode reward mean), and `time/fps`. The `ep_rew_mean` should increase over time as the agent learns.

## Verification
*   The training script runs without errors.
*   The `ep_rew_mean` in the training output shows a general increasing trend, indicating the agent is learning to achieve higher rewards (i.e., moving closer to the target).
*   A `simple_legged_robot_policy.zip` file is saved, containing the trained model.

## Challenge Questions
1.  **Reward Shaping**: Modify the `_compute_reward` function in `SimpleLeggedRobotEnv`. Add a penalty for joint angles being too close to their limits (e.g., `-np.sum(np.clip(np.abs(self.joint_angles) - (np.pi/2 - 0.1), 0, None)) * 0.1`). How does this affect the learning process and the final behavior?
2.  **Target Randomization**: In `reset()`, randomize the `target_pos` more broadly. Does the agent generalize better, or does it struggle to learn a single target?
3.  **Hyperparameter Tuning**: Experiment with `learning_rate`, `gamma`, and `clip_range` in the PPO model. How do these hyperparameters affect the speed and stability of learning?
4.  **Extend the Robot**: If you were to integrate this with a more realistic simulator like Isaac Lab, how would you map the `step` and `reset` functions of `SimpleLeggedRobotEnv` to Isaac Lab's API?
