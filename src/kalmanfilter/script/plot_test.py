#!/usr/bin/env python
from sslib import *
from nav_msgs.msg import Path
from std_msgs.msg import Header 


path = Path()


def vicon_callback(msg):
    path.header = Header()
    path.header.frame_id = "vicon"
    path.header.stamp = rospy.Time.now()
    path.poses.append(msg)
    pub.publish(path)

if __name__ == '__main__':
    rospy.init_node('plot', anonymous=True)

    rospy.Subscriber("/viconXbee_node/mocap/pose", PoseStamped, vicon_callback)
    pub = rospy.Publisher('path_vicon', Path, queue_size=0)

    rospy.spin()

