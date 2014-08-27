---
layout:             post
image:              /robot-web-tools.jpg
title:              Robot Web Tools with Baxter
demo:               https://github.com/jonrovira/robot_web_tools
requirements:       [ros-hydro-ros-base (basic ROS installation), 
                    ros-hydro-rosbridge-server (rosbridge_server),
                    mjpeg_server,
                    Rethink Robotics Baxter robot,
                    Properly configured workspace for Baxter robot]
overview:           Demonstrate the capabilities of Robot Web Tools, particularly when 
                    interfacing it with Rethink Robotics' Baxter robot. The included demo contains code to publish a simple message to a ROS Topic, stream one of Baxter's camera feeds, and control Baxter's left arm, al via a web browser.
demo-instructions:  ["Make sure 'Enable Networking' is unchecked in the networking menu of your computer",
                     "Ensure that Baxter's ethernet cable is connected to your computer",
                    ["Open up a terminal and run Avahi's network address configuration daemon", "$ sudo avahi-autoipd eth0"],
                    ["In a new terminal, run the Baxter setup script (assuming it's properly configured) that is located in the root directory of your ROS workspace", "$ ./baxter.sh"],
                    ["Start roscore", "$ roscore"],
                    ["In a new terminal, launch a rosbridge server", "$ roslaunch rosbridge_server rosbridge_websocket.launch"],
                    ["In a new terminal, start a mjpeg server", "$ rosrun mjpeg_server mjpeg_server"],
                    ["In Google Chrome, navigate to the index html page from the demo repository", "file:///path/to/file/index.html"],
                    ["To see Baxter's left hand camera stream, in a new terminal, run the startcamera script", "$rosrun robot_web_tools startcamera.py"]]
places-to-start:    [["Robot Web Tools", "http://robotwebtools.org/"],
                     ["Basic ROS functionality with roslibjs example", "http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality"],
                     ["MJPEG Single stream canvas example", "http://wiki.ros.org/mjpegcanvasjs/Tutorials/CreatingASingleStreamCanvas"]]
---


