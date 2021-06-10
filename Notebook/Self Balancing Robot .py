#!/usr/bin/env python
# coding: utf-8

# MD Muhaimin Rahman 

# Freelance Robotics and Computer Vision Engineer at Upwork . 
# Contact: sezan92@gmail.com 

# # Introduction
# 

# In this tutorial , I am going to build a Self Balancing RobotUrdf from Scratch in Gazebo. Self Balancing robots are one of the most fascinating Applications of Inverted Pendulum theory. [Click Here](https://en.wikipedia.org/wiki/Inverted_pendulum) to learn about them. This tutorial assumes that the reader has basic knowledge about ROS nodes,topics,publishing , subscribing,actions, services. If not , I advise to checkout these [videos](https://www.youtube.com/watch?v=xgLETnSMMYA) .

# ### Let's start with the model file named robot3.urdf.xacro
# [ Located at urdf folder]

# <?xml version = "1.0" ?>
# <robot name = "Self_Balancing_Robot" xmlns:xacro ="http://ros.org/wiki/xacro" >
# .....
# </robot>

# In the above code, it is saying that, the file is actually written in xml language. The <robot> tag is showing that whatever is inside of it, is the description of the robot. I think the name is self explanatory. As for xmlns:xacro, I won't explain it write now.. Please bear with me in this tutorial ;)
# 
# 

#     <link name = "Chassis">
# 		<inertial>
# 			<origin xyz = "0.0375 0 0" rpy = "0 0 0" />
# 			<mass value = "0.388"/>
# 			<inertia ixx ="0.0113" iyy=".01094" izz=".00971" ixy="0" ixz ="0" iyz = "0"/>
# 		</inertial>
# 		<collision>
# 			<geometry>
# 				<box size = "0.075 0.175 0.157" /> <!-- 0.075 0.175 0.157 -->
# 				 
# 			</geometry>
# 			<origin xyz = "0 0 0" rpy = "0 0 0"/>
# 			
# 		</collision>
# 		<visual>
# 			<xacro:Chassis_Mesh />
# 			
# 		
# 			
# 
# 		</visual>
# 	</link>

# link tag is the building block of a Robot model . Every part of the robot you want to work with, is defined as Links. It has three elements , inertial, collision, visual
# 
# - inertial element describes the inertia like Center of mass, mass, mass moment of inertia. The origin tag inside inertial is the co ordinates of Center of Mass of the link i.e. Chassis. 
# 
# - collision is the element which defines how the robot should act in the simulation world. Like a Box , Sphere or Cylinder ? Or Like a mesh file you will describe ? In the above codes , we have said it will behave like a box of size 0.075x0.175x0.157 meter cube . The origin tag below the geometry tag specifies the co-ordinates of the link w.r.t Gazebo Frame of Reference. 
# - visual This tag referes to how will you see in the simulation. You can copy the entire Collision code here. But wait ! There's a xacro tag ! What is it  ??
# 

# # Xacro

# Xacro is what made the whole code smaller than before! It is like a function of c++ code. In the beginning of the file robot3.urdf.xacro you'll see some lines like 

#     <xacro:macro name = "Chassis1" >
# 			<geometry>
# 				<box size = "0.075 0.175 0.157" /> <!-- 0.075 0.175 0.157 -->
# 				 
# 			</geometry>
# 	</xacro:macro>

# It defines a functionish code for this model , which tells the Gazebo server that whenever we will call xacro:Chassis1 it will work like the lines of code inside of it. So the above codeblock can be written, as following

#     `<link name = "Chassis">
# 		<inertial>
# 			<origin xyz = "0.0375 0 0" rpy = "0 0 0" />
# 			<mass value = "0.388"/>
# 			<inertia ixx ="0.0113" iyy=".01094" izz=".00971" ixy="0" ixz ="0" iyz = "0"/>
# 		</inertial>
# 		<collision>
# 			<xacro:Chassis1 /><!--
# 			<origin rpy = "0 0 0" xyz ="0 0 0.1101"/> -->
# 			<xacro:Chassis_origin /> 
# 			
# 		</collision>
# 		<visual>
# 			<xacro:Chassis_Mesh />
# 			
# 		
# 			
# 
# 		</visual>
# 	</link>

# ## Joint 
# 

# Joint Tag is something which joins a link to another link . There are several types of joints. 
# 
# - Fixed . It defines the joint as unmoveable.
# - Revolute. It says that the link can rotate upto certain limit
# - Continuous . It says that the link can rotate without any limit
# - Prismatic. It says the link can move along one axis. 
# - Floating . It says that the link can rotate in 6  axes of Freedom.
# - Let's look at two types of joints defined in the file.

#     <joint name="base_link_joint" type="fixed">
# 		<origin xyz = "0 0 0.001" rpy = "0 0 0" />
# 		<parent link="base_footprint"/>
# 		<child link="Chassis" />
# 	
# 	</joint> 

# - parent link is the boss of the joint :D child should explain himself
# - origin is the co ordinates of the joint with respect to parent's frame of reference. (Told you, he is the boss)
# - type = "fixed" means , the chassis will be fixed with the base_footprint

# 	<joint name ="Right_wheel_Joint" type = "continuous">
# 		<parent link = "base_footprint"/>
# 		<child link = "Right_Wheel"/>
# 		<origin xyz = "-0.0135 0.0875 -0.0785" rpy ="0 0 0.0"/>
# 		<limit effort="100000" velocity="100000"/>
#         <axis xyz="0 1 0"/>
# 	</joint>

# ### Quiz 
# - What type of joint?
# - Which link is the boss ?
# - Origin of Joint ? 

# Here you'll see two new tags named limit and axis. 
# - limit defines the limit of upper rotation, lower rotation (both in radians), maximum force , maximum velocity of a joint
# - Here we don't need to set limit for the joint rotations. So I haven't specified. otherwise it would be 
#     limit lower= "0" upper = "1.57" ...
# - the effort is for maximum force or torque in N or Nm respectively
# - So what is velocity specifying ? Any guess ? 
# 

# ## Plugin
# 

# Plugin is the link between this model and your ROS code. It says the simulation world what how the model should behave whenever you run the code. At the end of the model xacro file you'll notice a plugin like the following
# 

#  <gazebo> 
#     	<plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
#       	<legacyMode>false</legacyMode>
#       	<alwaysOn>true</alwaysOn>
#       	<updateRate>10</updateRate>
#       	<leftJoint>Left_wheel_Joint</leftJoint>
#       	<rightJoint>Right_wheel_Joint</rightJoint>
#       	<wheelSeparation>0.175</wheelSeparation>
#       	<wheelDiameter>0.0632</wheelDiameter>
#       	<torque>10</torque>
#       	<commandTopic>cmd_vel</commandTopic>
#       	<odometryTopic>odom</odometryTopic>
#       	<odometryFrame>odom</odometryFrame>
#       	<robotBaseFrame>base_footprint</robotBaseFrame>
# 
#     </plugin>
#   </gazebo>
# 

# As it's name suggests, it makes the robot Baseframe move like [Differential Drive](https://en.wikipedia.org/wiki/Differential_wheeled_robot) Movement. The leftJoint and RightJoint are to specified as Left wheel and Right Wheel of the robot. The commandTopic is the real Key. The name inside the topic defines the topic for the movement of the robot. Whatever message published in this topic . For more information please visit [This link](http://wiki.ros.org/geometry_msgs).
# 
# ## Quiz 
# - Can you say the tasks of other tags in the above gazebo plugin ? 

# # Homework
# Please try to open the xacro file and Answer the following questions. If you can't answer , relax! It's not an exam. Google them :) 

# - What are the functions of other joints and links 
# - What is the function of this tag  "xacro:Chassis_Mesh" ? 
# - What is Mesh file ? 
# - What are the functions of other two plugins ? 
# 

# I am planning to expand this notebook in future. Please let me know if you have any other question so that I can know what more confuision you have. Please feel free to contact via email  at sezan92[at]gmail.com

# # Conclusion 

# In this notebook I have tried to explain how to build a self balancing robot urdf from scratch. In other codes I have tried to explain how to use Control theories namely famous, PID, Fuzzy Logic and LQR controllers. For any confusion please dont hesitate to contact me .

# In[ ]:




