#!/usr/bin/env python
# coding: utf-8

# ### MD Muhaimin Rahman
# contact: sezan92@gmail.com
# 

# # Introduction

# In this notebook, I am describing the implementation of Control Systems namely [PID](#pid_sec),[Fuzzy Logic](#fuzzy_sec) and [LQR](#lqr_sec) on our robot. The $pidcontrol.py$ file has been used for our project. I haven't written this library. I have downloaded from this github project.[1](#pid)

# #### Importing Libraris

# In[1]:


import sys
import pidcontrol as pid
import numpy as np
import pybullet as p
import math
import time
import pybullet_data
import control 
import scipy


# <a id = "pid_sec"></a>
# ### PID

# The class below , "SelfBalance" does the main job. In this class, we see , one subscriber for cmd_vel topic which moves the robot. Also four subscriber functions. One subscriber function for accessing Imu values. The other subscriber functions to tune the PID gains . For PID controller function I have used a python code named $pidcontrol.py$ from the project at [1](#pid)

# In[2]:


class SelfBalance:
    def __init__(self):
        self.xvelMin=-.01
        self.xvelMax =0
        self.yMin = -0.01
        self.yMax = -0.001
        self.yPrev =0
        self.delY = 0
        self.Kp = 25
        self.Ki = 0.8
        self.Kd = 0.1
        self.controller=pid.PID_Controller(self.Kp,self.Ki,self.Kd)
        
    def callback(self,data):
        
        setPoint = 0
        y = data[1][1]*180/3.1416#theata changes along 1
        self.delY = y-self.yPrev
        
        if self.delY>self.yMax:
            self.yMax = self.delY
        elif self.delY<self.yMin:
            self.yMin = self.delY
        
        xvel = -self.controller.getCorrection(setPoint,y)
        if xvel>self.xvelMax:
            self.xvelMax=xvel
        elif xvel<self.xvelMin:
            self.xvelMin = xvel
        
        if xvel >26:
            xvel =26
        elif xvel<-26:
            xvel =-26
        
        self.yPrev = y
        
        return xvel
        #print "Max vel " + str(self.xvelMax) + " & Min vel " + str(self.xvelMin) + " Max delY " + str(self.yMax*180/3.1416) +" & Min delY" + str(self.yMin*180/3.1416)
    def callback_Kp(self,data):
        self.Kp = data.data
        self.controller=pid.PID_Controller(self.Kp,self.Ki,self.Kd)
    def callback_Ki(self,data):
        self.Ki = data.data
        self.controller=pid.PID_Controller(self.Kp,self.Ki,self.Kd)
    
    def callback_Kd(self,data):
        self.Kd = data.data
        self.controller=self.controllerpid.PID_Controller(self.Kp,self.Ki,self.Kd)
        


# In[3]:


balance=SelfBalance()
balance.callback([(0.0, 0.0, 0.0), [ 0., -0.,  0.]])


# Main Function

# ### Video

# [Youtube Link](https://youtu.be/kssD8unnWls)

# In[ ]:


id = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.8)

robot = p.loadURDF("../urdf/self_balance.urdf" , [0,0,0.2])

# num = p.getNumJoints(robot)
# for i in range(num):
#     info = p.getJointInfo(robot, i)
#     print(info,end="\n")
#     link_name = info[12].decode("ascii")
#     if link_name == "left_wheel": left_wheel = j
#     if link_name == "right_wheel": wheel_foot = j
left_joint=0
right_joint=1

print("----------------------------------------------------------------------------------------------------------------")
# print("Dynamic Info of Base : ",p.getDynamicsInfo(robot, -1),end="\n")
# #0->mass , 3->local inertial pos
# print("Base position and Orientation : " , p.getBasePositionAndOrientation(robot),end="\n")
# #1->orientation

# com = p.getDynamicsInfo(robot, -1)[3][2]
# com += p.getBasePositionAndOrientation(robot)[0][2] 
# print("Centre of mass - ", com)

#information required yaw
#imu sensor , kp ,ki ,kd
#set cmd_vel 

balance=SelfBalance()
while(True):
    position,orientation=p.getBasePositionAndOrientation(robot)
    euler_angles=np.array(p.getEulerFromQuaternion(orientation))#1->orientation
    deg_orien=euler_angles*180/np.pi
    #print(deg_orien)
    theta=deg_orien[1]
    #pos=position[0]
    velocity,angular=p.getBaseVelocity(robot)
    #print([velocity,euler_angles])
    vel=balance.callback([velocity,euler_angles])
    print(vel)
    p.setJointMotorControl2(robot, left_joint , p.VELOCITY_CONTROL, targetVelocity = vel)
    p.setJointMotorControl2(robot, right_joint , p.VELOCITY_CONTROL, targetVelocity = -vel)
    p.stepSimulation()
    time.sleep(0.01)


# ## Conclusion

# In this project , I have tested three famous controllers on Self Balancing Robot Simulation. I am planning to work on other controllers as well, like Fuzzy PID, Adaptive Fuzzy, Neural Network, Sliding motion controller etc. I also intend to work on other control system problems as well. Control System is fun! Please contact me for any confusions or bugs. 
# 
# #### MD Muhaimin Rahman
# 

# # Reference

# <a id ="pid"></a>
# [1] PyQuadSim [Repository](https://github.com/simondlevy/PyQuadSim/blob/master/pidcontrol.py)
# <br/>
# <a id ="fuzzy"></a>
# [2] Scikit-fuzzy DOI link 10.5281/zenodo.1002946
# <br/>
# <a id ="PythonControl"></a>
# [3] Python Control Package , [link](https://python-control.readthedocs.io/en/latest/)
# <br/>
# <a id ="SystemEquation"></a>
# [4] Control System Tutorials , [link](http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling)
