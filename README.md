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
The goal of this project is to get Baxter to stuff stockings that are hung on a wall with presents that are scattered across a table. He will identify a stocking, locate the corresponding present, and then place the present in the stocking.


<a name="Equipment and Hardware Requirements"></a>
###Equipment and Hardware Requirements
1. Baxter  
2. [ROS Indigo](http://wiki.ros.org/ROS/Installation) on Ubuntu 14.04  
3. Asus Xtion Pro Camera  
4. 4 Stockings  
5. 4 Presents of different colors  
6. 4 AR tags  
7. Table to place presents  
8. Wall and hangers to place stockings  

<a name="Preliminary Steps"></a>
###Preliminary Steps 
#####Setup Baxter  
Rethink Robotics has [Baxter Setup](http://sdk.rethinkrobotics.com/wiki/Baxter_Setup) instructions and [Workstation Setup](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) instructions.   
#####Setup demo area
There needs to be a table and presents within Baxter's reachable workspace

<a name="Project Overview"></a>
###Project Overview 
Outline of the steps that went into building the package that will run Baxter through the program  

1. Sweep the stockings  
2. Store the tag ID and location from each stocking  
3. Relate the stocking tag ID to a present color  
4. Sweep the table  
5. Identify the colors and locations of presents  
6. Move Baxter's gripper to the present location  
7. Pick up the present  
8. Move Baxter's gripper to the corresponding stocking location  
9. Drop the present into the stocking  


<a name="Implementation"></a>
###Implementation 



Below is what the launch file looks like:  
```
<launch>

</launch>
```

<a name="Dependencies"></a>
###Dependencies 
- `ar_track`

<a name="Package Installation"></a>
###Package Installation 
The following packages need to be installed
- `ar_track_alvar $ sudo apt-get install

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

- `sweep_stocking.py` commands Baxter to look at the stockings and store the stocking locations and corresponding presents  

- `sweep_table.py` commands Baxter to look at the table with presents and store the present locations   

- `vision_test.py`   


<a name="Conclusions"></a>
###Conclusions 



![Baxter Stocking Stuffer picture](https://raw.githubusercontent.com/ChuChuIgbokwe/ME495-Final-Project-Baxter-Stocking-Stuffer/master/baxterpic.jpeg)

