---
id: lab-1-ros2-dev-env
title: 'Lab 1: Setting up Your ROS 2 Development Environment'
sidebar_label: 'Lab 1: ROS 2 Dev Env'
---

# Lab 1: Setting up Your ROS 2 Development Environment

## Objective
The goal of this lab is to establish a functional ROS 2 Humble development environment. You will create a ROS 2 workspace, a custom package, and compile and run a simple "Hello World" ROS 2 publisher and subscriber.

## Theoretical Background
**ROS 2 (Robot Operating System 2)** is not an operating system, but a set of software libraries and tools that help you build robot applications. It provides a standardized way for different parts of a robot's software (nodes) to communicate with each other (topics, services, actions) using a DDS (Data Distribution Service) middleware.

*   **Workspace**: A directory where you store and build ROS 2 packages.
*   **Package**: The fundamental unit of ROS 2 software, containing nodes, libraries, configuration files, and launch files.
*   **Node**: An executable process that performs a specific task (e.g., a sensor driver, a motor controller, a planning algorithm).
*   **Topic**: A named channel over which nodes exchange messages (publish/subscribe communication).

## Prerequisites
*   **WSL2/Ubuntu 22.04 LTS**: As per Chapter 6.
*   **ROS 2 Humble**: Installed on your Ubuntu system.
    *   If not installed, follow the official documentation: [docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
*   **VS Code/Cursor**: With the Dev Containers extension configured.

## Step-by-Step Instructions

### Step 1: Open Your Development Container
1.  Launch VS Code or Cursor.
2.  Open the root folder of this book project (`Physical-AI-Humanoid-Robotics-Book`).
3.  If prompted, click "Reopen in Container" to launch your pre-configured Dev Container. If not prompted, open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P`) and select "Dev Containers: Reopen in Container".
4.  Once the container is running and your terminal is open, you should be inside a Linux environment with ROS 2 Humble sourced. Verify by typing:
    ```bash
    printenv | grep ROS_DISTRO
    ```
    You should see `ROS_DISTRO=humble`.

### Step 2: Create a ROS 2 Workspace
It's good practice to create a dedicated workspace for your custom packages.
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### Step 3: Create a Custom ROS 2 Package
We'll create a Python package named `my_robot_controller`.
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_controller
```

### Step 4: Write a Simple Publisher Node
Navigate into your new package's directory (`~/ros2_ws/src/my_robot_controller`).
Create a file named `my_robot_controller/publisher_member_function.py` and add the following Python code:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2! Count: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args) # Initialize ROS 2 communication
    minimal_publisher = MinimalPublisher() # Create the node
    rclpy.spin(minimal_publisher) # Keep the node alive
    minimal_publisher.destroy_node() # Destroy node on shutdown
    rclpy.shutdown() # Shutdown ROS 2 communication

if __name__ == '__main__':
    main()
```

**Code Explanation**:
1.  `rclpy.init(args=args)`: Initializes the ROS 2 Python client library.
2.  `MinimalPublisher(Node)`: Our node inherits from `rclpy.node.Node`.
3.  `super().__init__('minimal_publisher')`: Initializes the node with the name `minimal_publisher`.
4.  `self.create_publisher(String, 'topic', 10)`: Creates a publisher that sends `String` messages on a topic named `'topic'` with a queue size of 10.
5.  `self.create_timer(timer_period, self.timer_callback)`: Sets up a timer to call `timer_callback` every `0.5` seconds.
6.  `self.publisher_.publish(msg)`: Publishes the message.
7.  `rclpy.spin(minimal_publisher)`: Keeps the node running until interrupted.

### Step 5: Write a Simple Subscriber Node
In the same `my_robot_controller` package, create a file named `my_robot_controller/subscriber_member_function.py` and add the following Python code:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Code Explanation**:
1.  `self.create_subscription(String, 'topic', self.listener_callback, 10)`: Creates a subscriber that listens for `String` messages on the `'topic'` topic. When a message arrives, it calls `self.listener_callback`.
2.  `self.get_logger().info(...)`: Prints the received message to the console.

### Step 6: Update `setup.py` and `package.xml`
These files define your package's metadata and executables.

**Update `~/ros2_ws/src/my_robot_controller/setup.py`**:
Add the `entry_points` section. Locate the `data_files` section and add these lines:
```python
entry_points={
    'console_scripts': [
        'talker = my_robot_controller.publisher_member_function:main',
        'listener = my_robot_controller.subscriber_member_function:main',
    ],
},
```

**Update `~/ros2_ws/src/my_robot_controller/package.xml`**:
Add the following dependencies if they are not already there (they should be generated by default):
```xml
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
```

### Step 7: Build Your Package
Navigate back to your workspace root and build your package:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_controller
```
If the build is successful, you will see output indicating your package was built.

### Step 8: Source the Workspace and Run Nodes
You need to "source" your workspace to make your new executables available to ROS 2.
```bash
. install/setup.bash
```
Now, open *two new terminal tabs* in your Dev Container. In each new tab, you will also need to source the setup file:
```bash
# In each new terminal tab
. ~/ros2_ws/install/setup.bash
```

In the **first new terminal tab**, run the publisher:
```bash
ros2 run my_robot_controller talker
```

In the **second new terminal tab**, run the subscriber:
```bash
ros2 run my_robot_controller listener
```

You should see the publisher sending "Hello ROS 2!" messages and the subscriber printing "I heard!" messages.

## Verification
*   Publisher prints "Publishing: ..." messages.
*   Subscriber prints "I heard: ..." messages.
*   Run `ros2 topic list` in a third terminal. You should see `/topic` listed.
*   Run `ros2 node list`. You should see `minimal_publisher` and `minimal_subscriber` listed.

## Challenge Questions
1.  Change the `timer_period` in the publisher to `0.1` seconds. Rebuild and see the effect.
2.  Change the topic name in both nodes and `setup.py`. What happens if you don't change both?
3.  Implement a simple service in the `my_robot_controller` package. (Hint: Use `ros2 service call` to test it).
