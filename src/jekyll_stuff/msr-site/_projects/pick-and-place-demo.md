---
layout:            project
image:             /baxter-pick-place2.jpg
title:             Pick and Place Baxter Demo
demo:              https://github.com/jonrovira/baxter_work
requirements:      [OpenCV ROS package (cv2 and CvBridge),
                   Rethink Robotics Baxter robot,
                   Properly configured workspace for Baxter robot]
overview:          Use OpenCV to detect an object in Baxter's workspace and implement
                   motion and graps planning to manipulate one of Baxter's arms to grasp the object.
demo-instructions: ["Make sure 'Enable Networking' is unchecked in the networking menu of your
                   computer",
                   "Ensure that Baxter's ethernet cable is connected to your computer",
                   ["Open up a terminal and run Avahi's network address configuration daemon", "$ sudo avahi-autoipd eth0"],
                   ["In a new terminal, run the Baxter setup script (assuming it's properly configured) that is located in the root directory of your ROS workspace", "$ ./baxter.sh"],
                   ["Run the demo", "$ roslaunch 'pkg_name' pick_and_place.launch"]]
places-to-start:   [["Thresholding images by color example",
                   "opencv-srf.blogspot.ro/2010/09/object-detection-using-color-seperation.html"],
                   ["Worked Example of Visual Servoing", "sdk.rethinkrobotics.com/wiki/Worked_Example_Visual_Servoing"]]
---           