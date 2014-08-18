#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "../include/v_repConst.h"

int main(int argc, char* argv[])
{
	int leftMotorHandle;
	int rightMotorHandle;
	int sensorHandle;
	if (argc>=4)
	{
		leftMotorHandle = atoi(argv[1]);
		rightMotorHandle=atoi(argv[2]);
		sensorHandle=atoi(argv[3]);
	}
	

	// Create ROS node
	int _argc = 0;
	char** _argv = NULL;
	struct timeval tv;
	unsigned int timeVal=0;
	if (gettimeofday(&tv,NULL)==0)
		timeVal=(tv.tv_sec*1000tv.tv_usec/1000)&0x00ffffff;
	std::string nodeName("rosBubbleRob");
	std::string randId(boost::lexical_cast<std::string>(timeVal+int(999999.0f*(rand()/(float)RAND_MAX))));
	nodeName+=randId;
	ros::init(_argc, _argv,nodeName.c_str());

	if(!ros::master::check())
		return(0);

	ros::NodeHandle node("~");
	printf("vrep_test just started with node name %s\n",nodeName.c_str());
}