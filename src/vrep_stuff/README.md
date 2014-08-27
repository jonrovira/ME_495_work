# V-REP Stuff

###= Get your workstation set up
* Download v-rep from www.coppeliarobotics.com/downloads.html
* Download everyting in this directory

## vrep_baxter_test.py

#### Get the demo running
In one terminal, start roscore:
'''
$  roscore
'''
In another terminal, navigate to the directory where v-rep is located and start v-rep:
'''
$  ./vrep.sh
'''
Move the executable in this repository (on your machine) to the v-rep directory
'''
$  mv /path/to/this/repository/vrep_baxter_test.py /path/to/vrep/directory/
'''
In the v-rep window that opened, in the toolbar at the top, select File > Open Scene... and navigate to this repository on your machine. Select Baxter_Test.ttt.
In another terminal, start the demo
'''
$  rosservice call /vrep/simRosStartSimulation
'''
