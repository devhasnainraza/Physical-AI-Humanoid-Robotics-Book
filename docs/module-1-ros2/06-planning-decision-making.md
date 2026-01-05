---
id: planning-decision-making
title: '1.6 Planning & Decision Making'
sidebar_label: '1.6 Planning'
description: 'Behavior Trees in ROS 2 using BehaviorTree.CPP.'
---

# 1.6 Planning & Decision Making

**"Plan your work, then work your plan."**

We use **Behavior Trees (BT)** for high-level logic. It's modular, readable, and standard in ROS 2 (Nav2 uses it).

## 🎯 Lab Objectives
1.  **Install BehaviorTree.CPP**.
2.  **Create a simple Tree**: "Patrol" -> "Low Battery?" -> "Recharge".

---

## 1.6.1 The Logic (XML)

Save this as `main_tree.xml`:

```xml
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Fallback name="root">
            <Sequence name="RechargeSequence">
                <Condition ID="IsBatteryLow"/>
                <Action ID="GoToCharger"/>
            </Sequence>
            <Action ID="Patrol"/>
        </Fallback>
    </BehaviorTree>
</root>
```

*   **Fallback**: Try first child (Recharge). If it fails (Battery NOT low), do second child (Patrol).
*   **Sequence**: If Battery Low (Success), THEN Go To Charger.

---

## 1.6.2 The Code (C++)

```cpp
#include "behaviortree_cpp_v3/bt_factory.h"

// Custom Action
class Patrol : public BT::SyncActionNode {
  public:
    Patrol(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override {
        std::cout << "Patrolling..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

// ... (Define IsBatteryLow and GoToCharger similarly)

int main() {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<Patrol>("Patrol");
    // ... register others
    
    auto tree = factory.createTreeFromFile("./main_tree.xml");
    tree.tickRoot(); // Execute once
    return 0;
}
```

---

## 1.6.3 Quiz

1.  **In a Fallback node, if Child 1 returns FAILURE, what happens?**
    *   a) The whole tree fails.
    *   b) It executes Child 2.
    *   *Answer: b*

2.  **In a Sequence node, if Child 1 returns FAILURE, what happens?**
    *   a) The Sequence returns FAILURE immediately.
    *   b) It executes Child 2.
    *   *Answer: a*