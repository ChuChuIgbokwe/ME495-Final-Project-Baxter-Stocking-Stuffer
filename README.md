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
The goal of this project is to get a [Baxter Robot](http://www.rethinkrobotics.com/baxter/) to act as Santa's [elf](http://en.wikipedia.org/wiki/Christmas_elf) helper.  Baxter will have a set of presents and a set of stockings in his workspace.  Baxter will identify a stocking, locate the corresponding present, and then place the present in the stocking.


<a name="Equipment and Hardware Requirements"></a>
###Equipment and Hardware Requirements
1. Baxter Robot  
2. [ROS Indigo](http://wiki.ros.org/ROS/Installation) on Ubuntu 14.04  
3. [Asus Xtion Pro Camera](http://www.asus.com/Multimedia/Xtion_PRO/)  
4. [4 Stockings](http://www.amazon.com/Plush-Christmas-Stocking-White-Trim/dp/B001L11PQ0/ref=sr_1_9?ie=UTF8&qid=1416682045&sr=8-9&keywords=christmas+stockings)  
5. 4 Presents of different colors  
6. [4 AR tags](http://wiki.ros.org/ar_track_alvar?action=AttachFile&do=view&target=markers0to8.png)  
7. Table to place presents  
8. Wall and hangers to place stockings  

<a name="Preliminary Steps"></a>
###Preliminary Steps 
#####Setup Baxter  
Rethink Robotics has [Baxter Setup](http://sdk.rethinkrobotics.com/wiki/Baxter_Setup) instructions and [Workstation Setup](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) instructions.   
#####Setup demo area
There needs to be a table and presents within Baxter's reachable workspace  
![Baxter Workspace](https://raw.githubusercontent.com/ChuChuIgbokwe/ME495-Final-Project-Baxter-Stocking-Stuffer/master/baxter_workspace_placeholder.jpeg)  


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
10. Repeat steps 1-9 until Baxter has stuffed all of the stockings appropriately


<a name="Implementation"></a>
###Implementation 


Below is what the launch file looks like. It starts each of the nodes in the sequence in which they will be used. This is the launch file that you will need to run to run the full sequence. The other launch file in the folder is for simply starting up and testing the ar_track package in order to find ID numbers.

```
<launch>

  <!-- Include launch file that starts AR tracker and identifies stocking -->
  <include file="$(find baxter_stocking_stuffer)/launch/ar_trackv2.launch"/>


  <!-- Node to open Baxter's left hand camera, after closing them all -->
  <node pkg="baxter_tools" type="camera_control.py" name="close_left_camera" output="screen"  args="-c left_hand_camera">
  </node>
  <node pkg="baxter_tools" type="camera_control.py" name="close_right_camera" output="screen"  args="-c right_hand_camera">
  </node>
  <node pkg="baxter_tools" type="camera_control.py" name="close_head_camera" output="screen"  args="-c head_camera">
  </node>

  <node pkg="baxter_tools" type="camera_control.py" name="open_left_camera" output="screen"  args="-o left_hand_camera -r 640x400">
  </node>


  <!-- Launch the tracking node -->
  <node pkg="visp_auto_tracker" type="visp_auto_tracker" name="visp_auto_tracker" output="log">
    <param name="model_path" type="string" value="$(find visp_auto_tracker)/models" />
    <param name="model_name" type="string" value="pattern" />
    <param name="debug_display" type="bool" value="True" />
        
    <remap from="/visp_auto_tracker/camera_info" to="/cameras/left_hand_camera/camera_info"/>
    <remap from="/visp_auto_tracker/image_raw" to="/cameras/left_hand_camera/image"/>
  </node>


  <!-- Node that accepts a PoseStamped() message and moves toward it, if a solution is possible. -->
  <node pkg="baxter_stocking_stuffer" type="baxtermovement.py" name="movement_node" output="screen" >
  </node>

  <!-- Once stocking has been identified, determines the pose of the stocking using QR code -->
  <node pkg="baxter_stocking_stuffer" type="poseusingidandqrcode.py" name="stocking_pose" output="screen" >
  </node> -->

  <!-- Node to publish center of object after thresholded with OpenCV -->
  <node pkg="baxter_stocking_stuffer" type="open_cv_vision.py" name="vision_node" output="screen" >
  </node>

  <!-- Node listening to center of object that publishes PoseStamped() message for Baxter's gripper to get within grapsing reach -->
  <node pkg="baxter_stocking_stuffer" type="poseusingcolordetection.py" name="color_node" output="screen" >
  </node>


  <!-- Node that moves Baxter's gripper and held present back to stocking, then releases into stocking -->
  <node pkg="baxter_stocking_stuffer" type="back_to_stocking_and_release.py" name="returns_present" output="screen" >
  </node>


</launch>

```

<a name="Dependencies"></a>
###Dependencies:
- `ar_track`
- `visp_auto_tracker`
- ` `
- ` `

<a name="Package Installation"></a>
###Package Installation:
The following packages need to be installed
- `ar_track_alvar $ sudo apt-get install`

<a name="Future Improvements"></a>
###Future Improvements 


<a name="Other Scripts"></a>

###Main Scripts:
These nodes run in a certain sequence of steps. The way this is accomplished is by having each of these nodes listen to certain topics that contain boolean messages of true and false. Published messages of true to certain topics begin specific actions and the opposite is true for when false messages are published.

- `needed_present_identifier.py` 

Overall function: This node kicks off the stocking stuffing sequence. In addition, it is needed to identify whose present Baxter needs to search for on the table. 

Published topics:

/color_identifier
/scanned_stocking_id
/start/sweep
/start/stockingpose
/baxter_movement/posestamped

When an ID that has not had a present associated with it yet is found 

color identifier
scanned stocking id
start sweep; starts off as true and changes to false at the end once stocking id is identified
start stockingpose; published to true
baxter movement stamped

Subscribers:
ar pose marker
start/sweep
visp auto tracker

- `baxtermovement.py`

Overall function:

Publishers:

Subscribers:

 
- `open_cv_vision.py`

Overall function:

Publishers:

Subscribers:


- `poseusingcolordetection.py`

Overall function:

Publishers:

Subscribers:

- `back_to_stocking_and_release.py`

Overall function:

Publishers:

Subscribers:

- `poseusingidandqrcode.py`

Overall function:

Publishers:

Subscribers:

<a name="Conclusions"></a>
###Conclusions: 
####Future Steps:


![Baxter Stocking Stuffer picture](https://raw.githubusercontent.com/ChuChuIgbokwe/ME495-Final-Project-Baxter-Stocking-Stuffer/master/baxterpic.jpeg)

