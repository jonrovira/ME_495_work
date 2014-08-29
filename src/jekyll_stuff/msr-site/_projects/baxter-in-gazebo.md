---
layout:             project
image:              /baxter-in-gazebo2.png
title:              Baxter in Gazebo
author:             Jarvis Schultz
date:               August 29th, 2014
demo:               https://github.com/jonrovira/ME_495_work
requirements:       [python-wstool,
                    python-rosdep,
                    ros-hydro-pcl-conversions,
                    ros-hydro-control-msgs,
                    ros-hydro-cmake-modules,
                    ros-hydro-qt-build,
                    ros-hydro-moveit-full,
                    ros-hydro-driver-common,
                    ros-hydro-image-common,
                    ros-hydro-rostest-gazebo,
                    Baxter Simulation repository (must request access by emailing RSDK.support@rethinkrobotics.com]
overview:           Learn how to quickly get a Baxter simulation running in Gazebo.
                    Execute the joint position keyboard example in the simulation.
demo-instructions:  ["Complete the simulator installation instructions located at
                    sdk.rethinkrobotics.com/wiki/Simulator_Installation",
                    ["Run the Baxter setup script located in the root of your workspace with the simulation parameter", "$ ./baxter.sh sim"],
                    ["Start the simulation with controllers", "$ roslaunch baxter_gazebo baxter_world.launch"],
                    ["In a new terminal, run the joint position keyboard example", "$ rosrun baxter_examples joint_position_keyboard.py"]]
places-to-start:    [["Baxter Simulation Installation",
                    "http://sdk.rethinkrobotics.com//wiki/Simulator_Installation"]]
---


