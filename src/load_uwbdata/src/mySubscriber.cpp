#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"

#include "load_uwbdata/uwbdata.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"

using namespace std;

void chatterCallback(load_uwbdata::uwbdata msg)
{
	ROS_INFO("The distance is: %f", msg.distance);
}

void viconCallback(geometry_msgs::PoseStamped msg)
{
	ROS_INFO("The position is [%.3f %.3f %.3f]", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "mySubscriber");
	ros::NodeHandle n;
	
	ros::Subscriber sub1 = n.subscribe("/myPublisher/uwb_distance", 1, chatterCallback);
	ros::Subscriber sub2 = n.subscribe("/viconXbee_node/mocap/pose", 1, viconCallback);
	
	ros::spin();

	return 0;
}

