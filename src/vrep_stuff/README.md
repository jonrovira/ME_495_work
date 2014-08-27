# V-REP Stuff
Includes two demo scripts for keyboard control of Rethink Robotics' Baxter robot and Kuka's Youbot within the v-rep simulation. The vrep_baxter_test.py script is slightly more refined than the vrep_youbot_test.py script, so you may find a few better code practices there, especially for collecting user keyboard input.

In order to connect each python script to v-rep, the child scripts within the Baxter_Test.ttt and Youbot_Test.ttt scenes, contain a very small amount of code calling the appropriate python executables. You can view that code in the v-rep window.


### Get your workstation set up
* Install ROS
* Download v-rep from www.coppeliarobotics.com/downloads.html
* Download everyting in this directory


## vrep_baxter_test.py

#### Description
vrep_baxter_test.py is a simple demo that facilitates keyboard control of the Baxter robot within the v-rep simulation. The basic control loop involves collecting a user's keyboard input, searching a binding dictionary for what action the input maps to, and commanding Baxter to complete that action. The script demonstrates how to utilize ROS with v-rep via ROS services and ROS publishers/subscribers.

#### Get the demo running
In one terminal, start roscore:
```
$  roscore
```
In another terminal, navigate to the directory where v-rep is located and start v-rep:
```
$  ./vrep.sh
```
Move the executable in this repository to the v-rep directory
```
$  mv /path/to/this/repository/vrep_baxter_test.py /path/to/vrep/directory/
```
In the v-rep window that should now be open, in the toolbar at the top, select
```
File > Open Scene... and navigate to this repository on your machine. Select Baxter_Test.ttt.
```
In another terminal, start the demo
```
$  rosservice call /vrep/simRosStartSimulation
```
When you're ready to stop the demo, you can run
```
$  rosservice call /vrep/simRosStopSimulation
```


## vrep_youbot_test.py

#### Description
Similar to vrep_baxter_test.py, the vrep_youbot_test.py sript demonstrates keyboard control of the Youbot within v-rep. The control loop functions the same way. Slightly different from the baxter script, both position and velocity control are implemented by setting different JointSetStateData().setModes.data values.

#### Get the demo running
In one terminal, start roscore:
```
$  roscore
```
In another terminal, navigate to the directory where v-rep is located and start v-rep:
```
$  ./vrep.sh
```
Move the executable in this repository to the v-rep directory
```
$  mv /path/to/this/repository/vrep_youbot_test.py /path/to/vrep/directory/
```
In the v-rep window that should now be open, in the toolbar at the top, select
```
File > Open Scene... and navigate to this repository on your machine. Select Youbot_Test.ttt.
```
In another terminal, start the demo
```
$  rosservice call /vrep/simRosStartSimulation
```
When you're ready to stop the demo, you can run
```
$  rosservice call /vrep/simRosStopSimulation
```
