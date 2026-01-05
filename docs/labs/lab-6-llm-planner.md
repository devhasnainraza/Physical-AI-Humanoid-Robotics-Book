--- 
id: lab-6-llm-planner
title: 'Lab 6: Implementing an LLM Planner for Robotic Actions'
sidebar_label: 'Lab 6: LLM Planner'
---

# Lab 6: Implementing an LLM Planner for Robotic Actions

## Objective
This lab brings together concepts from **Chapter 6: Planning and Decision Making** and **Chapter 11: Future of Physical AI**. You will implement an LLM-based planner that interprets high-level human commands (e.g., "Go to the kitchen and fetch a soda") and translates them into a sequence of low-level robot actions. We will use the **ReAct (Reason + Act)** prompting strategy to make the planner robust.

## Theoretical Background
Traditional robotics planning relies on predefined state machines or planning algorithms (like A*). LLM planners offer a more flexible, human-like way to interpret complex commands.
*   **ReAct Prompting**: A strategy where the LLM interleaves **Thought** (internal monologue) and **Action** (calling predefined tools/functions). This allows the LLM to reason, execute actions, observe their outcomes, and iteratively refine its plan, similar to how humans solve problems.
*   **Tool Use / Function Calling**: Providing the LLM with a list of available robot "skills" (functions) it can call, along with their descriptions and parameters. The LLM's task is then to choose and sequence these tools.

## Prerequisites
*   **Python 3.10+**: With `pip`.
*   **OpenAI API Key**: Or access to Gemini API / Anthropic API. (We'll use OpenAI for this lab). Set `OPENAI_API_KEY` environment variable.
    ```bash
    export OPENAI_API_KEY="YOUR_API_KEY"
    ```
*   **Install necessary libraries**:
    ```bash
    pip install openai # For OpenAI's Python client
    ```

## Step-by-Step Instructions

### Step 1: Define Robot Actions (Tools)
First, we define the "tools" our robot has. In a real ROS 2 system, these would map to ROS 2 actions or services. For this lab, we'll represent them as Python functions that just print what the robot would do.

1.  Create a new Python file named `~/ros2_ws/src/my_llm_planner/robot_actions.py` (you might need to create the `my_llm_planner` package first with `ros2 pkg create --build-type ament_python my_llm_planner`).

```python
# ~/ros2_ws/src/my_llm_planner/robot_actions.py

def navigate_to(location: str):
    """
    Moves the robot to a specified named location.
    Args:
        location (str): The name of the location (e.g., "kitchen", "bedroom", "charging_station").
    """
    print(f"[ROBOT_ACTION] Navigating to: {location}")
    # In a real system, this would call a ROS 2 Nav2 action client
    return f"Successfully navigated to {location}"

def find_object(object_name: str):
    """
    Rotates the robot's head/camera to look for a specified object.
    Args:
        object_name (str): The name of the object to find (e.g., "soda can", "keys").
    Returns:
        str: JSON string with object's 3D coordinates if found, else "Object not found".
    """
    print(f"[ROBOT_ACTION] Looking for: {object_name}")
    # Simulate finding the object
    if object_name == "soda can":
        return '{{"object_name": "soda can", "x": 0.5, "y": 0.2, "z": 0.8}}'
    else:
        return 'Object not found'

def pick_up(x: float, y: float, z: float):
    """
    Commands the robot arm to pick up an object at given 3D coordinates.
    Args:
        x (float): X coordinate of the object relative to the robot base.
        y (float): Y coordinate of the object relative to the robot base.
        z (float): Z coordinate of the object relative to the robot base.
    """
    print(f"[ROBOT_ACTION] Picking up object at ({x:.2f}, {y:.2f}, {z:.2f})")
    # Simulate picking up
    return "Object picked up successfully"

def speak(text: str):
    """
    Makes the robot speak a given text aloud.
    Args:
        text (str): The text for the robot to say.
    """
    print(f"[ROBOT_ACTION] Robot says: '{text}'")
    # In a real system, this would use a Text-to-Speech service
    return "Spoke successfully"

# Map tool names to actual functions
TOOLS = {
    "navigate_to": navigate_to,
    "find_object": find_object,
    "pick_up": pick_up,
    "speak": speak
}
```

### Step 2: Implement the LLM Planner with ReAct Strategy
Now we create the main planner that uses these tools.
1.  Create a file named `~/ros2_ws/src/my_llm_planner/llm_planner_node.py` and add the following Python code:

```python
import os
import json
import openai
from my_llm_planner.robot_actions import TOOLS # Import our tools

# Set up OpenAI API key
openai.api_key = os.getenv("OPENAI_API_KEY")

class LLMPlanner:
    def __init__(self):
        self.model_name = "gpt-4-turbo" # Or "gpt-3.5-turbo", "gpt-4" for cheaper/faster
        self.client = openai.OpenAI()

    def generate_react_prompt(self, user_command, available_tools):
        """Generates a ReAct-style prompt for the LLM."""
        tool_descriptions = "\n".join([
            f"- {name}({', '.join(f'{k}: {v.__annotations__[k].__name__}' for k, v in tool.__annotations__.items() if k != 'return')}) -> {tool.__doc__.strip()}"
            for name, tool in available_tools.items()
        ])

        system_message = f"""
        You are a highly intelligent robot planner named Alfred. Your goal is to translate high-level human commands into a sequence of executable robot actions.

        You have access to the following tools:
        {tool_descriptions}

        To use a tool, respond in this exact JSON format:
        {{
            "action": "tool_name",
            "args": {{"param1": "value1", "param2": "value2"}}
        }}

        You should use the following format for your responses:
        Thought: You should always think about what to do.
        Action: {{...}} (tool call in JSON)
        Observation: The result of the action.
        ... (this Thought/Action/Observation cycle repeats)
        Thought: I have completed the task.
        Action: "task_complete"

        Begin!

        Example:
        User: Move to the bedroom.
        Thought: The user wants me to navigate to the bedroom. I should use the navigate_to tool.
        Action: {{"action": "navigate_to", "args": {{"location": "bedroom"}}}} 
        Observation: Successfully navigated to bedroom
        Thought: I have completed the task.
        Action: "task_complete"
        """
        
        return system_message

    def plan(self, user_command, max_iterations=5):
        """
        Generates and executes a plan for the robot using the LLM and available tools.
        """
        messages = [
            {"role": "system", "content": self.generate_react_prompt(user_command, TOOLS)},
            {"role": "user", "content": user_command}
        ]
        
        for i in range(max_iterations):
            print(f"\n--- Iteration {i+1} ---")
            
            # Get LLM response
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=messages,
                stop=["Observation:"] # Stop when LLM is about to output an observation
            )
            
            llm_output = response.choices[0].message.content
            print(f"LLM Output:\n{llm_output}")
            messages.append({"role": "assistant", "content": llm_output})

            # Extract Thought and Action
            thought_prefix = "Thought:"
            action_prefix = "Action:"
            
            thought = ""
            action_json_str = ""

            # Parse thought and action
            if thought_prefix in llm_output:
                thought_start = llm_output.find(thought_prefix) + len(thought_prefix)
                thought_end = llm_output.find(action_prefix, thought_start)
                if thought_end != -1:
                    thought = llm_output[thought_start:thought_end].strip()
                else:
                    thought = llm_output[thought_start:].strip()
            
            if action_prefix in llm_output:
                action_start = llm_output.find(action_prefix) + len(action_prefix)
                action_json_str = llm_output[action_start:].strip()

            if not action_json_str:
                print("Error: Could not parse action JSON from LLM output.")
                messages.append({"role": "user", "content": "Error: Could not parse action. Please provide valid JSON action."})
                continue
            
            try:
                action_data = json.loads(action_json_str)
            except json.JSONDecodeError:
                print(f"Error decoding JSON: {action_json_str}")
                messages.append({"role": "user", "content": f"Error: Invalid JSON action: {action_json_str}. Please provide valid JSON."})
                continue

            # Execute action
            if action_data == "task_complete":
                print("[PLANNER] Task completed by LLM.")
                break
            
            tool_name = action_data.get("action")
            tool_args = action_data.get("args", {})
            
            if tool_name in TOOLS:
                print(f"[PLANNER] Executing tool: {tool_name} with args: {tool_args}")
                observation = TOOLS[tool_name](**tool_args)
            else:
                observation = f"Error: Tool '{tool_name}' not found."
            
            print(f"Observation: {observation}")
            messages.append({"role": "user", "content": f"Observation: {observation}"})
            
        else:
            print("[PLANNER] Max iterations reached, task not completed.")
        
        return messages


def main():
    planner = LLMPlanner()
    
    while True:
        user_command = input("\nUser Command (type 'exit' to quit): ")
        if user_command.lower() == 'exit':
            break
        
        planner.plan(user_command)

if __name__ == '__main__':
    main()

```

### Step 3: Update `setup.py` and `package.xml` for `my_llm_planner`
**Update `~/ros2_ws/src/my_llm_planner/setup.py`**:
```python
from setuptools import setup

package_name = 'my_llm_planner'

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
            'llm_planner = my_llm_planner.llm_planner_node:main',
        ],
    },
)

```

**Update `~/ros2_ws/src/my_llm_planner/package.xml`**:
Add these dependencies:
```xml
  <depend>python3-openai</depend>
  <!-- Other dependencies if you were to integrate with ROS 2 -->
```

### Step 4: Build Your Package and Run the Planner
Navigate back to your workspace root and build your package:
```bash
cd ~/ros2_ws
colcon build --packages-select my_llm_planner
```
Source your workspace:
```bash
. install/setup.bash
```
Now, run your LLM planner:
```bash
ros2 run my_llm_planner llm_planner
```
The script will prompt you for commands. Try:
*   `Go to the kitchen.`
*   `Find the soda can in the kitchen.`
*   `Pick up the soda can.`

## Verification
*   The script successfully starts and prompts for input.
*   The LLM generates `Thought` and `Action` steps in response to your command.
*   The simulated `robot_actions.py` functions are called and print their output.
*   The LLM continues to plan based on the `Observation` from the executed actions until "task_complete".

## Challenge Questions
1.  **Error Handling**: Modify the `find_object` function to sometimes return "Object not found". How does the LLM react? Can you improve the LLM's system prompt to better handle such failures (e.g., "If an object is not found, try navigating to another common location for it")?
2.  **New Tool**: Add a new tool, `charge_battery()`, to `robot_actions.py` and describe it in the prompt. Test if the LLM can use this new tool appropriately when given a command like "Charge yourself."
3.  **Real-time Integration**: Discuss how you would integrate this Python planner with a live ROS 2 system. How would `robot_actions.py` functions interact with ROS 2 services and actions? (Hint: You would create ROS 2 action clients/service clients within `robot_actions.py`).
4.  **Observation Feedback**: Instead of just passing back `Successfully navigated`, pass more detailed observations (e.g., "Navigated to kitchen, but path was obstructed once"). How does this richer observation affect the LLM's subsequent planning?
