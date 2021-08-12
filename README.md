# Why  LQR if we already have PID? 

The application of industrial robots has greatly increased in recent  years making production systems more and more efficient. With this in  mind, increasingly efficient controllers are needed for those robots. 

![image](https://user-images.githubusercontent.com/56964828/126338592-12198b94-8ab3-4935-a49a-062eda016648.png)

The LQR control technique represents a intermediate control solution between the simple techniques, such as PID, and more complex ones, such as predictive control, when it comes to design and equation .

Whereas sometimes, LQR controllers are robust and produce a very low steady state error, but with a big transition delay and using multiple feedback gains, that makes them a bad choice when the system needs fast parameters update and has no direct access to all states of the plant. On the other hand, a PID controller gives a faster response but not with robust gains as the previous controller. [[1]](https://fei.edu.br/~psantos/PID2737725.pdf)

So the best controller depends on the problem and which one might perform well is something that one gains by experience .

![image](https://user-images.githubusercontent.com/56964828/126339069-5a8d4ada-bd69-4a7b-bef6-8c3043feead7.png)

# All roads lead to inverted pendulum .

#### Pendulums are Everywhere

![Real -world application of an inverted pendulum - the Segway](https://www.quanser.com/wp-content/uploads/2019/09/Segway-200x300.jpg)

*The inverted pendulum DOES represent many real-world systems*. Examples include the Segway, the human posture systems, the launching  of a rocket, and so on. Basically, any system that requires vertical  stabilization has dynamics that are similar to an inverted pendulum.  Sure, the dynamics in these real-world systems are more complex, but  they are related. The work involved in modeling and controlling an  inverted pendulum carries over to many engineering areas. [[2]](https://www.quanser.com/blog/why-is-the-pendulum-so-popular/)

Also the maths is easy .

# Solution

Check out the solutions folder .

Self_Balance_withLQR.py : Simple standing bot on 2 wheels 

Self_Balance_withLQR2.py : Bot on 2 wheels with extra feature of up/down/right/left movement  

Self_Balance_withPID.py : Standing bot using PID 

solution.webm : Video  of Self_Balance_withLQR2.py 

File Structure :

1. exercise : This folder contains the incomplete code for LQR and PID
2. experiment : Some example codes to get familiar with pybullet environment
3. solution : Solution code for LQR and PID 
4. urdf : 2-wheel bot urdf

Implementation of LQR , PID for self balancing bot. 