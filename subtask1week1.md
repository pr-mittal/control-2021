Assesment for this part be like,

   [![img](https://camo.githubusercontent.com/7ff1d9b63e7f6f41fbd822dedda2692d4747b07bc4723d5b287a1be8b9dbc2cd/68747470733a2f2f656e637279707465642d74626e302e677374617469632e636f6d2f696d616765733f713d74626e3a414e64394763524937776135697764314c7851764b4664575056426441545f55684b656f74583934535866555a53434639736d74374d78332673)](https://camo.githubusercontent.com/7ff1d9b63e7f6f41fbd822dedda2692d4747b07bc4723d5b287a1be8b9dbc2cd/68747470733a2f2f656e637279707465642d74626e302e677374617469632e636f6d2f696d616765733f713d74626e3a414e64394763524937776135697764314c7851764b4664575056426441545f55684b656f74583934535866555a53434639736d74374d78332673)

On a more serious note,**Control Theory** is one crucial element in robotics and needs quite the time to master. However, as  always we will aim to give you a head start with the fundamental  definitions.

## 

## What is Robot control ?

*Robotic control is the system that contributes to the movement of robots. Fairly Intuitive huh ?*_

*Q.So,is it a mechanical aspect / software aspect of the bot?*

*Ans.Simply the bridge between both !!*

Thus a controller involves the mechanical aspects and program systems that makes possible to control robots. Robotics could be controlled in  various ways, which includes using manual control, wireless control,  semi-autonomous (which is a mix of fully automatic and wireless  control), and fully autonomous (which is when it uses AI to move on its  own, but there could be options to make it manually controlled).But  controllers can be broadly classified based on their design and  irrespective of their field of deployment.

## 

## Types and topologies:

There are surplus material available to learn this in-depth but for  the light weight introduction we recommend the 5 part mini-video series  from **Matlab**

[Understanding Control Systems](https://in.mathworks.com/videos/series/understanding-control-systems-123420.html)

**Note:** This series is just for learning the theory, no need to worry about using Matlab as shown in the videos.



### 1. System

- A **system** is a **combination** or an arrangement **of different physical components** which act together as an entire unit to achieve certain objective.
- For example, **a classroom** or **a lamp**.

### 2. Control System

- To **control means to regulate**, to direct or to command. 

- Hence a **control system** is an arrangement of **different physical elements** connected in such a manner so as to regulate, direct or command itself or some other system.

- An **important classification** of control systems is as follows:

  - **Open loop systems**

    - A **system** in which output is dependent on input but **controlling action or input is totally independent of the output** or changes in the output of the system, is called an **Open Loop System**.
    - An open loop system is also **called an uncontrolled system**. 
    - The output of such a system is not controlled as **the system has no feedback**.
    - For example, **an electric toaster**. 
      - The quality of the toast totally depends on the timer the user sets and it is to be judged by the user.

  - **Closed loop systems**

    - A system in which the controlling action or **input is somehow dependent on the output** or changes in the output is called a **Closed Loop System**.

    - A closed loop system is also **called a controlled system**. 

    - The output of such a system is controlled **as it receives feedback**. **Proportional Integral Derivative** aka **PID** is an example of a close loop system.

    - For example, **a human being.** Yes you read correctly. The best example is us i.e. humans. Lets see how.

      - Suppose you want to pick up a book.

    - The first thing that you will do is to locate the book using your eyes. Right?

      - Once you locate the book, you will estimate the position of your hand with respect to the book.

    - This is **called as feedback**.

      - The difference in the positions is nothing but the distance your hand will have to move to grab the book.

    - This is **called as error**. 

      - Now, your brain will give signal to your hands.

    - This process will continue till you grab the book.

    - Need a **much more complicated example**? Watch this video by **KUKA - Robots & Automation** to understand the importance of control theory in robotics industry.

      <iframe width="560" height="315" src="https://www.youtube.com/embed/lv6op2HHIuM" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

      

    - Here is one more by **SpaceX** where they landed the first stage of a rocket on a droneship.

      <iframe width="560" height="315" src="https://www.youtube.com/embed/sYmQQn_ZSys" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

    - This what control systems are all about. You have **1000s of control systems** around you. 

    - Look around and determine **at least 3 control systems**.

    - Control Systems is a **huge** field of engineering and you have just started your journey.

    - Here are some **references** for you to get more information:

    - These resources cover material that is **typically taught over the course of one engineering semester**. 

    ### **Voilà !!**

    **You just completed the starter pack of control systems.** 

    ## References

    - **Control Theory** YouTube Playlist **by Brian Douglas**:

      <iframe width="560" height="315" src="https://www.youtube.com/embed/oBc_BHxw78s" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
    
    - Control Systems Course offered by **National Programme on Technology Enhanced Learning** (NPTEL) [here](https://nptel.ac.in/courses/107/106/107106081/).

### 3. Installation

Let's get started. First things first 

1. `pip install numpy`
2. `pip install slycot `
3. ``pip install control`
4. `pip install gym`

Read the documentation for control  [here](https://pypi.org/project/control/) and for gym [here](https://gym.openai.com/docs/)

Note : If you face any issue while installing slycot in Ubuntu

`sudo apt install libopenblas-dev`

`pip install slycot`

Also upgrade to latest numpy `pip install numpy --upgrade`

For pybullet errors refer [here](https://github.com/Robotics-Club-IIT-BHU/Robotics-Camp-2021/tree/main/Basics%20of%20pyBullet/Week%201/Subpart%201)

## Verification

Run this sample code to verify if everything is working

```
import gym
import control
import numpy as np
import pybullet_data
import control

env = gym.make('CartPole-v0')
env.reset()
for _ in range(1000):
    env.render()
    env.step(env.action_space.sample()) # take a random action
env.close()
```

