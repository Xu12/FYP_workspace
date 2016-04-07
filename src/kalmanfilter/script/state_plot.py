#!/usr/bin/env python
from sslib import *
from kalmanfilter.msg import state
from nav_msgs.msg import Path
from std_msgs.msg import Header 


path_vicon = Path()
path_uwb = Path()

def vicon_callback(msg):

    path_vicon.header = Header()
    path_vicon.header.frame_id = "vicon"
    path_vicon.header.stamp = rospy.Time.now()
    path_vicon.poses.append(msg)
    pub1 = rospy.Publisher('path_vicon', Path, queue_size=0)
    pub1.publish(path_vicon)

def uwb_callback(msg):
    
    path_uwb.header = Header()
    path_uwb.header.frame_id = "ned"
    path_uwb.header.stamp = rospy.Time.now()
    path_uwb.poses.append(msg)
    pub2 = rospy.Publisher('path_uwb', Path, queue_size=0)		
    pub2.publish(path_uwb)    

if __name__ == '__main__':
    rospy.init_node('plot', anonymous=True)

    rospy.Subscriber("/est_state", PoseStamped, uwb_callback)
    rospy.Subscriber("/viconxbee_node/mocap/pose", PoseStamped, vicon_callback)

    rospy.spin()


