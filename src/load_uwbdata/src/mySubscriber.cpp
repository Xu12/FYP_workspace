#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/Int64.h"

using namespace std;

void chatterCallback(const std_msgs::Int64::ConstPtr& msg)
{
	ROS_INFO("The distance is: %ld", msg->data);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mySubscriber");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
	
	ros::spin();

	return 0;
}

