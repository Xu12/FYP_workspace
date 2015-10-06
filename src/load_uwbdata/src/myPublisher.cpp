#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"

#include <sstream>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include "load_uwbdata/uwbdata.h"  //self-defined msg file

#include "ssuwb.h"

using namespace std;


int main(int argc, char *argv[])
{
    double distance;
    int node[4];
    geometry_msgs::Point position;


	ros::init(argc, argv, "myPublisher");
	ros::NodeHandle n("~");
	ros::Publisher publisher = n.advertise<load_uwbdata::uwbdata> ("uwb_distance", 1);

	n.getParam("node0id", node[0]);
	n.getParam("node1id", node[1]);
	n.getParam("node2id", node[2]);
	n.getParam("node3id", node[3]);

	std::string argv1,argv2;
	n.getParam("device", argv1);
	argv[1] = &argv1[0u];
	n.getParam("port", argv2);
	argv[2] = &argv2[0u];

	ssUWB(argv[1],argv[2]);


	geometry_msgs::Point node0_pos, node1_pos, node2_pos, node3_pos;
	n.getParam("node0_posx", node0_pos.x);
	n.getParam("node0_posy", node0_pos.y);
	n.getParam("node0_posz", node0_pos.z);
	n.getParam("node1_posx", node1_pos.x);
	n.getParam("node1_posy", node1_pos.y);
	n.getParam("node1_posz", node1_pos.z);
	n.getParam("node2_posx", node2_pos.x);
	n.getParam("node2_posy", node2_pos.y);
	n.getParam("node2_posz", node2_pos.z);
	n.getParam("node3_posx", node3_pos.x);
	n.getParam("node3_posy", node3_pos.y);
	n.getParam("node3_posz", node3_pos.z);


	int count = 0;
	
	while (ros::ok())
	{
		for (int i=0; i<4; i++)
		{
			distance = 1.0*uwb(node[i])/1000;

			switch(i)
			{
				case 0: position.x = node0_pos.x;
						position.y = node0_pos.y;
						position.z = node0_pos.z;
						break;
				case 1: position.x = node1_pos.x;
						position.y = node1_pos.y;
						position.z = node1_pos.z;
						break;
				case 2: position.x = node2_pos.x;
						position.y = node2_pos.y;
						position.z = node2_pos.z;
						break;
				case 3: position.x = node3_pos.x;
						position.y = node3_pos.y;
						position.z = node3_pos.z;
						break;
			}

			load_uwbdata::uwbdata msg;
			msg.distance = distance;
			msg.position = position;

			printf("msgCount: %d\nnodeID: %d\n", count, node[i]);
			printf("position: [%.3f  %.3f  %.3f]\n", position.x, position.y, position.z);
			printf("distance: %.3f\n", distance);
			printf("---\n");

			publisher.publish(msg);
		}

		++count;
	}

	return 0;
}
		
