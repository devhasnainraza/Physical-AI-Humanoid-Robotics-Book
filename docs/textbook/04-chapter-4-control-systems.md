---
id: chapter-4-control-systems
title: 'Chapter 4: Control Systems'
sidebar_label: '4. Control Systems'
description: 'From PID to Whole-Body Control: Making robots move smoothly and safely.'
---

# Chapter 4: Control Systems

**"Control theory is the art of making a robot obey, even when physics wants to disobey."**

## 🎯 Learning Objectives
By the end of this chapter, you will be able to:
1.  **Tune a PID controller** intuitively without guessing.
2.  **Understand "Stiffness" vs "Compliance"** and why stiff robots break things.
3.  **Implement Feedforward control** to make your robot defy gravity.

---

## 4.1 The Intuition: The Driver and the Car

Imagine you are driving a car. You want to maintain a speed of **60 km/h** ($x_{des}$). 
*   **The Error ($e$)**: The difference between 60 km/h and your current speed.
*   **The Actuator**: The gas pedal ($u$). 

### 4.1.1 PID Control: The Three Behaviors

**PID** stands for **Proportional, Integral, Derivative**. It combines three different driving styles:

1.  **P (Proportional) - "The Reactive Driver"**
    *   *Logic*: "I am slow, so I press the gas. The slower I am, the harder I press."
    *   *Math*: $u_p = K_p \cdot e(t)$
    *   *Problem*: As you get close to 60, you let go of the gas, so you might settle at 55 (Steady State Error).

2.  **I (Integral) - "The Patient Driver"**
    *   *Logic*: "I have been slow for a long time. I am getting annoyed. I will press the gas harder and harder until we hit 60."
    *   *Math*: $u_i = K_i \int e(t) dt$
    *   *Benefit*: Fixes the "55 vs 60" gap (Eliminates steady-state error).
    *   *Danger*: **Windup**. If you are stuck in mud, the driver pushes the pedal to the floor. When you get unstuck, you launch like a rocket (Overshoot).

3.  **D (Derivative) - "The Cautious Driver"**
    *   *Logic*: "Whoa, we are accelerating too fast! We are going to overshoot 60! Ease off!"
    *   *Math*: $u_d = K_d \frac{de(t)}{dt}$
    *   *Benefit*: Acts like a **Damper** or brake. Reduces oscillation.
    *   *Danger*: If your speedometer is jittery (Sensor Noise), the D-term panics.

### 4.1.2 The Full Equation
$$ u(t) = \underbrace{K_p e(t)}_{ \text{Spring}} + \underbrace{K_i \int e(t) dt}_{ \text{Bias Correction}} + \underbrace{K_d \dot{e}(t)}_{ \text{Damper}} $$

---

## 4.2 Student Support: How to Tune PID?

"My robot is shaking!" or "My robot is too slow!" — Here is the **Manual Tuning Guide**:

1.  **Set $K_i$ and $K_d$ to Zero.** Start with just P.
2.  **Increase $K_p$** until the robot oscillates (wobbles around the target) like a undamped spring.
3.  **Increase $K_d$** (Damping) until the oscillation stops. The robot should snap to the target and stick.
4.  **Check Error.** If the robot stops *near* the target but not *at* it (e.g., gravity pulls it down), slowly **increase $K_i$** to fix the gap.
    *   *Warning*: Keep $K_i$ very small. It causes instability easily.

---

## 4.3 Impedance Control: The Virtual Spring

**Classical Control (PID)** tries to force the robot to a position no matter what.
*   *Scenario*: You tell a PID arm to go through a table.
*   *Result*: It applies maximum torque, breaking the table or its motors.

**Impedance Control** makes the robot act like a spring.
*   *Scenario*: You tell an Impedance arm to go through a table.
*   *Result*: It pushes against the table with a force proportional to the depth ($F = Kx$), acting like a spring. It is **Compliant**.

### 4.3.1 The Equation
We don't control Position directly. We control the **Relationship** between Force and Position.

$$ F_{ext} = M_d \ddot{e} + D_d \dot{e} + K_d e $$

*   $K_d$: **Stiffness**. High = Metal bar. Low = Soft rubber.
*   $D_d$: **Damping**. High = Moving in honey. Low = Moving in air.

### 4.3.2 Real-World Use Case: Interaction
*   **Wiping a window**: You need to be stiff in the movement direction (Up/Down) but compliant in the normal direction (In/Out) so you don't break the glass.
*   **Walking**: When a leg hits the ground, it acts as a shock absorber (Low Stiffness) to handle impact, then stiffens up to push.

---

## 4.4 Feedforward Control: Cheating Physics

Feedback (PID) reacts to errors. **Feedforward** prevents them.

Imagine holding a 5kg weight with your arm extended.
*   **Feedback**: Your arm drops, your brain feels the error, and recruits muscles to pull it back up. (Reactive).
*   **Feedforward**: You *know* the weight is heavy. You recruit muscles *before* it drops. (Predictive).

**Gravity Compensation**:
$$ \tau_{total} = \text{PID}(e) + G(q) $$
We simply add the torque needed to cancel gravity ($G(q)$) directly to the motor command. This makes the robot feel "weightless" to the PID controller.

---

## 4.5 Operational Space Control (OSC)

Usually, we want to control the **Hand** (Task Space), not the **Joints** (Configuration Space).
Instead of calculating Inverse Kinematics (IK) -> Joint PID, we compute forces at the hand directly.

$$ F_{task} = \Lambda(q) \ddot{x}_{des} + \mu(q, \dot{q}) + p(q) $$
*   We calculate the force $F$ needed at the hand.
*   We convert $F$ to joint torques: $\tau = J^T F$.

**Why is this better?**
Because the math handles the physics of the *entire arm*. The controller knows that moving the shoulder is "heavier" than moving the wrist and adjusts gains automatically.

---

## 🛠️ Practical Implementation (Python)

```python
class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0
        self.prev_error = 0
        
    def update(self, target, current):
        error = target - current
        
        # P Term
        p_term = self.kp * error
        
        # I Term (with anti-windup clamp)
        self.integral += error * self.dt
        self.integral = max(min(self.integral, 10.0), -10.0) # Safety!
        i_term = self.ki * self.integral
        
        # D Term
        d_term = self.kd * (error - self.prev_error) / self.dt
        self.prev_error = error
        
        return p_term + i_term + d_term

# Example: Gravity Compensation
# tau = PID + m*g*cos(theta) * L_com
```

## 4.6 Quiz

1.  **Which term is responsible for "Overshoot"?**
    *   a) P (Proportional)
    *   b) I (Integral)
    *   c) D (Derivative)
    *   *Answer: b (and a if too high)*

2.  **Why do we prefer Impedance Control for walking robots?**
    *   a) It is faster.
    *   b) It allows the legs to absorb ground impacts like springs.
    *   c) It uses less battery.
    *   *Answer: b*
