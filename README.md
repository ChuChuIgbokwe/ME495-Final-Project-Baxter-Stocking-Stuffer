ME495-Final-Project-Baxter-Stocking-Stuffer
===========================================

Group: Sabeen Admani, Chukwunyere Igbokwe, Josh Marino, and Andrew Turchina

###Table of Contents
[Objective](#Objective)  
[Equipment and Hardware Requirements](#Equipment and Hardware Requirements)  
[Preliminary Steps](#Preliminary Steps)  
[Project Overview](#Project Overview)  
[Implementation](#Implementation)  
[Dependencies](#Dependencies)  
[Package Installation](#Package Installation)  
[Future Improvements](#Future Improvements)  
[Other Scripts](#Other Scripts)  
[Conclusions](#Conclusions)  


<a name="Objective"></a> 
###Objective
The goal of this project is to get Baxter to stuff stockings that are hung on a wall with presents that are scattered across a table. He wil identify a stocking, locate the corresponding present, and then place the present in the stocking.


<a name="Equipment and Hardware Requirements"></a>
###Equipment and Hardware Requirements
1. Baxter  
2. [ROS Indigo](http://wiki.ros.org/ROS/Installation) on Ubuntu 14.04  
3. Asus Xtion Pro Camera  


<a name="Preliminary Steps"></a>
###Preliminary Steps 
Setup Baxter  
Rethink Robotics has [Baxter Setup](http://sdk.rethinkrobotics.com/wiki/Baxter_Setup) instructions and [Workstation Setup](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) instructions.   


<a name="Project Overview"></a>
###Project Overview 
Outline of the steps that went into building the package


<a name="Implementation"></a>
###Implementation 


<a name="Dependencies"></a>
###Dependencies 


<a name="Package Installation"></a>
###Package Installation 


<a name="Future Improvements"></a>
###Future Improvements 


<a name="Other Scripts"></a>
###Other Scripts 
- `baxter_grip.py` controls the position of Baxter's gripper   

- `iknodebean.py` provides a   

- `iknodebean_test.py` was used as   

- `movement_visp.py`   

- `movement_visp_and_opencv.py`   

- `needed_present_identifier.py` listens to the color/tag node and selects the object needed  

- `sweep_stocking.py` commands Baxter to look at the stockings and store their locations and corresponding presents  

- `sweep_table.py` commands Baxter to look at the table with presents and identify which present to be grasped   

- `vision_test.py`   


<a name="Conclusions"></a>
###Conclusions 



![Baxter Stocking Stuffer picture](https://raw.githubusercontent.com/ChuChuIgbokwe/ME495-Final-Project-Baxter-Stocking-Stuffer/master/baxterpic.jpeg)

