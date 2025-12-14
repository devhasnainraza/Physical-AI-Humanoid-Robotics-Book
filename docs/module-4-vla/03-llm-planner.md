---
sidebar_position: 3
title: The LLM Planner
---

# Cognitive Planning with LLMs

## 1. Prompt Engineering for Robots

You cannot just ask ChatGPT "clean the room." You need to give it a **Function Interface** (Tools).

### 1.1 The System Prompt

```text
You are the brain of a humanoid robot named Cortex-H1.
You have access to the following low-level tools (skills):

1. navigate_to(location_name)
   - Moves the robot to a labeled location (kitchen, desk, charger).
2. find_object(object_name)
   - Rotates head to look for an object. Returns x,y,z if found.
3. pick_up(x, y, z)
   - Attempts to grasp object at coordinates.
4. speak(text)
   - Says text aloud.

User: "I'm thirsty, can you help?"
Context: User is sitting on the couch. There is a kitchen nearby.
Output: A JSON list of actions.
```

---

## 2. ReAct: Reason + Act

Simple prompting is often brittle. **ReAct** is a prompting strategy where the LLM intersperses thoughts with actions.

**User**: "Get me my water bottle."

**LLM Trace (ReAct)**:
1.  **Thought**: User wants water. I need to find it first.
2.  **Action**: `navigate_to("kitchen")`
3.  **Observation**: (Robot arrives at kitchen).
4.  **Thought**: I am in the kitchen. I need to see if the bottle is here.
5.  **Action**: `find_object("water bottle")`
6.  **Observation**: Found at (1.2, 0.5, 0.8).
7.  **Thought**: I see it. I can pick it up.
8.  **Action**: `pick_up(1.2, 0.5, 0.8)`

This allows the robot to handle failures (e.g., if Observation was "Object not found", the Thought would change to "Maybe it's in the bedroom").

---

## 3. Implementing the Planner in Python

We use **LangChain** to manage this structured output.

```python
from langchain.chat_models import ChatOpenAI
from langchain.schema import SystemMessage, HumanMessage
import json

llm = ChatOpenAI(model="gpt-4-turbo", temperature=0)

system_prompt = """
You are a robot planner. Output strictly valid JSON.
Available Actions: [navigate_to(loc), pick_up(obj), speak(text)].
"""

def plan_task(user_command):
    messages = [
        SystemMessage(content=system_prompt),
        HumanMessage(content=user_command)
    ]
    
    response = llm.predict_messages(messages)
    
    try:
        plan = json.loads(response.content)
        return plan
    except:
        return []

# Usage
cmd = "Bring me a soda from the fridge"
plan = plan_task(cmd)
# Executor loop would iterate through 'plan' and call ROS 2 services
```

---

## 4. The Capstone: Putting it All Together

In the final project, you will combine:
1.  **Whisper** (Hearing "Bring me a soda")
2.  **LLM** (Planning the sequence)
3.  **Nav2** (Driving to the kitchen)
4.  **Isaac ROS** (Seeing the soda)
5.  **Controller** (Grabbing it)

This is the holy grail of Embodied AI.