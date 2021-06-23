## Subpart 2

# Understanding Proportional Integral Derivative (PID) Controller

**NOTE:** Make sure you have covered the Control Systems document of subpart 1 before proceeding further.

## Introduction

- **Proportional-integral-derivative** (PID) controllers are **widely used** in industrial systems despite the significant developments of recent years in control theory and technology. 
- PID is an example of a **closed loop system**.
- They **perform well** for a wide class of processes. 
- Also, they give **robust performance** for a wide range of operating conditions and, they are easy to implement using analogue or digital hardware.
- PID is perhaps the most important,popular and simplistic **feed back controller**,out there. The significance of which could be done justice only by the videos below and *not even us !!*
- Refer this video by **Aerospace Controls Laboratory of Massachusetts Institute of Technology (MIT)**.

<iframe src="https://www.youtube.com/embed/4Y7zG48uHRo" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen="" style="z-index:5;position:relative;" width="710" height="450" frameborder="0"></iframe>

> **NOTE:** 
>
> - Try watching the video again to understand as much as possible. It has a lot of important information.
> - After you think you have some background about PID start reading below.

To put things in perspective, we will have to go through some definitions.

------

## Variations of PID controllers

### 1. Proportional Controller aka P Controller

- Proportional controller is mostly used **in first order processes** with single energy storage **to stabilize the unstable process**. 

- The main usage of the P controller is to **decrease the steady state error** of the system. 

- As the **proportional gain factor K** increases, the **steady state error** of the system **decreases**. 

- However, despite the reduction, **P control can never** manage to **eliminate** the **steady state error** of the system. 

- As we **increase the proportional gain**, it provides **smaller amplitude** and **phase margin**, **faster dynamics** satisfying wider frequency band and **larger sensitivity** to the noise. 

- We can **use** this controller **only** when our **system is tolerable** to a **constant steady state error**. 

- In addition, it can be easily concluded that applying **P controller decreases** the **rise time**.

- Moreover, **after** certain value of **reduction on the steady state error**, **increasing K** only **leads to overshoot** of the system response. 

- P control also **causes oscillation if sufficiently aggressive** in the presence of lags and/or dead time.

  > **Assignment**: Learn about dead time in control systems.

- The more lags (higher order), the more problem it leads. Plus, it directly **amplifies process noise**.

- A P controller consists of only a linear gain Kp. The output of such controller can be simply given as

- 
  
  ```bash
  output = Kp * error
  ```

### 2. Proportional-Derivate Controller aka PD Controller

- A controller which **changes the input** of the controller to **proportional plus derivative of error signal** is called **PD** controller.
- It is used to **damp the oscillations** that arise because of increasing the proportional constant.

- 
  
  ```bash
  output = (Kp * error) + (Kd * ((error - previous error)/Δt))  
  
  where, Δt is a small duration of time
  ```

### 3.  Proportional-Integral-Derivate Controller aka PID Controller

- A controller which **changes the input** of the controller to **proportional plus derivative of error signal plus the integral of the error** is called **PID** controller.
- It is used to **remove the steady state error** which might arise in PD controller.

```bash
sum err = sum err + error
output = (Kp * error) + (Kd * ((error - previous error)/Δt)) + (Ki * sum err * Δt)
```



More mathematically PID is represented as

# output =    [![img](https://camo.githubusercontent.com/e6a30bc3ca5bb6ddd88d3bc365f72a657c737decb4c3b4cd756e39d5770bee19/68747470733a2f2f656e637279707465642d74626e302e677374617469632e636f6d2f696d616765733f713d74626e253341414e64394763526d524475415a39336a376657663032464476754658704e416c6a4f59364f5a774b646d7a754c6d322d6f657270384c356726757371703d434155)](https://camo.githubusercontent.com/e6a30bc3ca5bb6ddd88d3bc365f72a657c737decb4c3b4cd756e39d5770bee19/68747470733a2f2f656e637279707465642d74626e302e677374617469632e636f6d2f696d616765733f713d74626e253341414e64394763526d524475415a39336a376657663032464476754658704e416c6a4f59364f5a774b646d7a754c6d322d6f657270384c356726757371703d434155)

1. Wanna C PID,..here it's in action..
   1. [Single motor target tracking](https://www.youtube.com/watch?v=fusr9eTceEo).
   2. [Self-balancing PID-ball tracker](https://www.youtube.com/watch?v=57DbEEBF7sE)

## References

1. The series of videos by **MATLAB** will help to grasp the concept of PID in the best way possible. You can watch the entire playlist from [here](https://www.youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y).

   <iframe src="https://www.youtube.com/embed/wkfEZmsQqiA" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen="" style="z-index:5;position:relative;" width="710" height="450" frameborder="0"></iframe>

2. This document by **National Programme on Technology Enhanced Learning** will help you get some great insights about PID. You can download it from [here](https://nptel.ac.in/content/storage2/courses/108105063/pdf/L-12(SS) (IA&C) ((EE)NPTEL).pdf).

## Task

# [CartPole-v1](https://gym.openai.com/envs/CartPole-v1/)

A pole is attached by an un-actuated joint to a cart, which moves along a frictionless track. The system is controlled by applying a force of +1 or -1 to the cart. The pendulum starts upright, and the goal is to prevent it from falling over. A reward of +1 is provided for every timestep that the pole remains upright. The episode ends when the pole is more than 15 degrees from vertical, or the cart moves more than 2.4 units from the center.

![image][https://takashi-matsushita.github.io/jekyll/assets/img/posts/CartPole.gif]

The code :



```python
import numpy as np
import gym

def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))

env = gym.make('CartPole-v1')
desired_state = np.array([0, 0, 0, 0])
desired_mask = np.array([0, 0, 1, 0])

P, I, D = 0.1, 0.01, 0.5

for i_episode in range(20):
    state = env.reset()
    integral = 0
    derivative = 0
    prev_error = 0
    for t in range(500):
        env.render()
        error = state - desired_state

        integral += error
        derivative = error - prev_error
        prev_error = error

        pid = np.dot(P * error + I * integral + D * derivative, desired_mask)
        action = sigmoid(pid)
        action = np.round(action).astype(np.int32)

        state, reward, done, info = env.step(action)
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            break
env.close()
```

