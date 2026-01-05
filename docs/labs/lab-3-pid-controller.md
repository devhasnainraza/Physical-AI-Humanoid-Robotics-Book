---
id: lab-3-pid-controller
title: 'Lab 3: Implementing a PID Controller for a Simulated Robot Joint'
sidebar_label: 'Lab 3: PID Controller'
---

# Lab 3: Implementing a PID Controller for a Simulated Robot Joint

## Objective
This lab provides a hands-on application of PID control theory (Chapter 4). You will implement a custom PID controller in Python and simulate its effect on a robot joint, observing how different gain values ($K_p, K_i, K_d$) affect the joint's movement towards a desired setpoint.

## Theoretical Background
A **PID (Proportional-Integral-Derivative) controller** is a feedback control loop mechanism widely used in industrial control systems and robotics. It continuously calculates an `error` value as the difference between a desired `setpoint` and a measured `process variable`. The controller attempts to minimize the error by adjusting the `control output`.

The three terms refer to:
*   **Proportional ($K_p$)**: Accounts for present values of the error.
*   **Integral ($K_i$)**: Accounts for past values of the error.
*   **Derivative ($K_d$)**: Accounts for possible future values of the error.

The output from the PID controller is the weighted sum of these three terms.

## Prerequisites
*   **ROS 2 Humble Development Environment**: Set up as in Lab 1.
*   **Python 3.10+**: With `numpy`.

## Step-by-Step Instructions

### Step 1: Create a New ROS 2 Package for Control
We'll create a Python package for our controller.
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_control
```

### Step 2: Implement the PID Controller Class
Navigate into your new package's directory (`~/ros2_ws/src/my_robot_control`).
Create a file named `my_robot_control/pid_controller.py` and add the PID class we discussed in Chapter 4:

```python
import time
import numpy as np

class PIDController:
    """
    A robust PID controller implementation for robotics applications.
    Includes anti-windup and derivative kick prevention.
    """
    def __init__(self, Kp, Ki, Kd, setpoint, output_limits=(-10.0, 10.0), windup_guard=20.0, sample_time=0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self.windup_guard = windup_guard
        self.sample_time = sample_time # Expected time between updates
        
        self.clear()

    def clear(self):
        """Resets the controller's state variables."""
        self.last_time = None
        self.last_error = 0.0
        self.integral = 0.0
        self.previous_measurement = 0.0 # Used for derivative on measurement

    def update(self, measurement):
        """
        Calculates the control signal based on the current measurement.
        """
        current_time = time.time()
        if self.last_time is None:
            self.last_time = current_time
            self.previous_measurement = measurement
            return 0.0 # No control action on first update

        dt = current_time - self.last_time
        
        # If dt is too small, skip update to prevent derivative noise amplification
        if dt < self.sample_time / 2.0: # Only update if sufficient time has passed
            return self.last_output # Return last computed output

        error = self.setpoint - measurement
        
        # Proportional term
        p_term = self.Kp * error
        
        # Integral term with anti-windup guard
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.windup_guard, self.windup_guard)
        i_term = self.Ki * self.integral
        
        # Derivative term: calculated on measurement to avoid derivative kick
        derivative = (measurement - self.previous_measurement) / dt
        d_term = self.Kd * -derivative # Negative sign for feedback

        # Calculate the raw output
        output = p_term + i_term + d_term
        
        # Clamp the output to defined limits
        output = np.clip(output, self.output_limits[0], self.output_limits[1])
        
        # Store state for next iteration
        self.last_error = error # For P and I
        self.previous_measurement = measurement # For D
        self.last_time = current_time
        self.last_output = output # Store last output for dt filtering
        
        return output

```

### Step 3: Simulate a Joint with ROS 2
We'll create a ROS 2 node that simulates a joint and applies the PID controller.
Create a file named `my_robot_control/joint_controller_node.py` and add the following code:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from my_robot_control.pid_controller import PIDController # Import our PID class

class JointControllerNode(Node):

    def __init__(self):
        super().__init__('joint_controller_node')
        
        # Declare parameters for PID gains and setpoint
        self.declare_parameter('setpoint', 0.0)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.05)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('output_min', -5.0)
        self.declare_parameter('output_max', 5.0)
        self.declare_parameter('windup_guard', 10.0)
        self.declare_parameter('sample_time', 0.01) # 100 Hz simulation

        # Get parameters
        setpoint = self.get_parameter('setpoint').get_parameter_value().double_value
        kp = self.get_parameter('kp').get_parameter_value().double_value
        ki = self.get_parameter('ki').get_parameter_value().double_value
        kd = self.get_parameter('kd').get_parameter_value().double_value
        output_min = self.get_parameter('output_min').get_parameter_value().double_value
        output_max = self.get_parameter('output_max').get_parameter_value().double_value
        windup_guard = self.get_parameter('windup_guard').get_parameter_value().double_value
        sample_time = self.get_parameter('sample_time').get_parameter_value().double_value

        self.get_logger().info(f"PID Controller for setpoint: {setpoint}, Kp:{kp}, Ki:{ki}, Kd:{kd}")

        # Initialize PID controller
        self.pid = PIDController(
            Kp=kp, Ki=ki, Kd=kd, setpoint=setpoint, 
            output_limits=(output_min, output_max), 
            windup_guard=windup_guard, 
            sample_time=sample_time
        )

        # Simulate a joint state
        self.current_joint_position = 0.0 # Starts at 0 degrees/radians
        self.joint_velocity = 0.0
        self.joint_acceleration = 0.0
        
        # Publishers
        self.position_pub = self.create_publisher(Float64, '/sim_joint/position', 10)
        self.control_pub = self.create_publisher(Float64, '/sim_joint/control_output', 10)

        # Timer to run the simulation loop
        self.timer = self.create_timer(sample_time, self.update_joint_state)

    def update_joint_state(self):
        # 1. Get current measurement
        measurement = self.current_joint_position

        # 2. Update PID and get control output
        control_output = self.pid.update(measurement)

        # 3. Simulate joint physics (very simple model: control_output acts like a force)
        # a = F/m, v = v + a*dt, p = p + v*dt
        mass = 1.0 # kg
        damping = 0.1 # friction
        
        force = control_output - (self.joint_velocity * damping) # Apply damping
        acceleration = force / mass
        
        self.joint_velocity += acceleration * self.pid.sample_time
        self.current_joint_position += self.joint_velocity * self.pid.sample_time

        # Publish current state and control output
        pos_msg = Float64()
        pos_msg.data = self.current_joint_position
        self.position_pub.publish(pos_msg)

        ctrl_msg = Float64()
        ctrl_msg.data = control_output
        self.control_pub.publish(ctrl_msg)
        
        self.get_logger().info(
            f"SetP:{self.pid.setpoint:.2f} | Current Pos:{self.current_joint_position:.2f} | Vel:{self.joint_velocity:.2f} | Ctrl:{control_output:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = JointControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Update `setup.py` and `package.xml`
**Update `~/ros2_ws/src/my_robot_control/setup.py`**:
Add the `entry_points` for the new node.
```python
entry_points={
    'console_scripts': [
        'joint_controller = my_robot_control.joint_controller_node:main',
    ],
},
```

**Update `~/ros2_ws/src/my_robot_control/package.xml`**:
Add `std_msgs` dependency if not already present.
```xml
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>numpy</depend>
```

### Step 5: Build Your Package
Navigate back to your workspace root and build your package:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_control
```

### Step 6: Source and Run the Controller
Source your workspace:
```bash
. install/setup.bash
```
Now, run your joint controller. We'll set the setpoint to 45 degrees (or radians, depending on your interpretation) and tune the gains.
```bash
ros2 run my_robot_control joint_controller --ros-args -p setpoint:=45.0 -p kp:=0.8 -p ki:=0.05 -p kd:=0.1
```
Observe the output in the terminal. The `Current Pos` should smoothly approach `45.0`.

### Step 7: Visualize with `rqt_plot` (Optional but Recommended)
Open a new terminal, source your workspace, and run:
```bash
rqt_plot /sim_joint/position /sim_joint/control_output
```
You will see a real-time plot of the joint's position and the control output from the PID controller. Experiment with different `kp`, `ki`, `kd` values to see how they affect the plot (e.g., higher Kp leads to faster response but more overshoot).

## Verification
*   The joint's `Current Pos` should converge to the `setpoint` in the terminal output.
*   The `rqt_plot` should show a smooth curve for position approaching the setpoint, and the control output should settle near zero once the setpoint is reached.

## Challenge Questions
1.  **Tuning**: Experiment with different `Kp`, `Ki`, `Kd` values. Can you make the joint reach the setpoint faster without significant overshoot? What happens if `Kp` is too high?
2.  **Integral Windup**: Set a very high `Ki` and a `setpoint` that is initially unreachable (e.g., `setpoint:=1000`). Observe the `control_output`. Once the setpoint becomes reachable, what happens? How does the `windup_guard` help?
3.  **Disturbance**: Modify the `update_joint_state` to add a small, constant `disturbance` force. How does the PID controller compensate?
4.  **Reference Tracking**: Change the `setpoint` dynamically during runtime (e.g., using a ROS 2 parameter update or a subscription to a new topic). How does the controller track the changing reference?
