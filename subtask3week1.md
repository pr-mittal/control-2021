## Subpart 3

Lot's of theory time to get our hands dirty with some strong application .

## Task for the part

Hope,you have gone through the above content about **PID** well, as far as this task goes, in a nutshell we implement a *PID controller*. So,most of you might have a intuitive understanding about PID for now.

Hence, the essential components will be

1. **A target point**- such that the error is 0 when setpoint == targetpoint
2. **A Sensor**- to find the current *setpoint*.
3.  **Actuators/Motors:** - to actually apply a action and try decrease the error.

Thus,in this task we will try implementing a PID controller to control the turning motion of the car about direction of motion of car so as to make the car follow the path in the direction stated .It could be better  understood from the video below.

<iframe src="https://www.youtube.com/embed/4Y7zG48uHRo" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen="" style="z-index:5;position:relative;" width="710" height="450" frameborder="0"></iframe>

Thus,in our case.

1. **A target point**- is to make the car follow the path via a smooth turn (decrease the distance from the track ).
2. **A Sensor**- is the  current location of object in world space i.e. how much deviated from the current alignment.
3. **Actuators/Motors:** - is the motors of the car,that makes the car align.

Hence the error can be easily calculated as,
$$
Line : y=mx+c
$$
   error = perpendicular distance of the car from the line 
$$
error = |y-mx-c|/sqrt(1+m^2)
$$
The starter code [pid_control_starter.py](https://github.com/NiranthS/Robo-Summer-Camp-20/blob/master/Part3/Subpart3/pid_control_starter.py), has the necessary structure for the code,with the following user inputs alredy implemented,

1. **right arrow**- anti clockwise rotation about z axis,essentially to give a initial disturbance and switch on PID to correct the alignment.
2. **left arrow** - clockwise rotation about z axis,,essentially to give a initial disturbance and switch on PID to correct the alignment.
3. **c key**      - to start PID control
4. **r key**      - to stop PID control and go back to manual control to cause a error. 4.Thus,you are expected
5. to get the location of car using the getLinkState . Read documentation [here](https://usermanual.wiki/Document/pybullet20quickstart20guide.479068914.pdf)
6. calculate the error and use PID to align your car back from the **initial user created disturbance** when *the key c* is pressed.

# Further Reading

## Improving your Controller

- The simple equations stated **may not** always **be enough** to stabilize the system. 
- Hence there are **various techniques to improve the performance** of PID controller. These are as follows:
  - Sample Time
  - Derivative Kick
  - On-The-Fly-Tuning Changes
  - Reset Windup Mitigation
  - On/Off
  - Initialization
  - Controller Direction
  - Proportional on measurement
- You can read about these in the **blog by brettbeauregard** [here](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/).

> **NOTE:** The above blog is very important to write your code. Make sure you read everything carefully.

That's for PID controller . See you next week . We will be working on LQR controller.

------

​														**403B48484H1E3D4744443B3C474A3B4C41463H1E1F1F**

​														(What does this mean 🤔. Try to decode as FUN 😄.)

​																				May the *PID* be with you! 😛

------

