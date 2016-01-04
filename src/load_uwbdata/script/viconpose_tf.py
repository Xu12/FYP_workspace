#!/usr/bin/env python

import time
import rospy
import tf
import tf2_ros
import math
from std_msgs.msg import Header
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, PoseWithCovarianceStamped


def frame_tf(msg):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "ned"
    t.child_frame_id = "vicon"
    t.transform.translation.x=0
    t.transform.translation.y=0
    t.transform.translation.z=0
    q = tf.transformations.quaternion_from_euler(math.pi, 0, -53.0*math.pi/180)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    br.sendTransform(t)

    t2 = TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "vicon"
    t2.child_frame_id = "uav"
    t2.transform.translation.x = msg.pose.position.x
    t2.transform.translation.y = msg.pose.position.y
    t2.transform.translation.z = msg.pose.position.z
    t2.transform.rotation.x = msg.pose.orientation.x
    t2.transform.rotation.y = msg.pose.orientation.y
    t2.transform.rotation.z = msg.pose.orientation.z
    t2.transform.rotation.w = msg.pose.orientation.w
    br.sendTransform(t2)


if __name__ == '__main__':
    rospy.init_node('tf_frame')
    rospy.Subscriber('/viconxbee_node/mocap/pose', PoseStamped, frame_tf)
    pub = rospy.Publisher('/mavros/vision_pose/vision', PoseStamped, queue_size=0)	
    pub_tar = rospy.Publisher('/mavros/setpoint_position/local_position', PoseStamped, queue_size=0)
    rate = rospy.Rate(10)
    tfBuffer = tf2_ros.Buffer()
    listener = tf.TransformListener()
    for i in xrange(10):
        rate.sleep()
    while not rospy.is_shutdown():    
        try:
            (trans,rot) = listener.lookupTransform('ned', 'uav', rospy.Time(0))
            msg_tf = PoseStamped()
            msg_tf.header = Header()
            msg_tf.header.frame_id = "ned"
            msg_tf.header.stamp = rospy.Time.now()

            msg_tf.pose.position.x = trans[0]
            msg_tf.pose.position.y = trans[1]
            msg_tf.pose.position.z = trans[2]

            msg_tf.pose.orientation.x = rot[0]
            msg_tf.pose.orientation.y = rot[1]
            msg_tf.pose.orientation.z = rot[2]
            msg_tf.pose.orientation.w = rot[3]
            
            pub.publish(msg_tf)

            (trans,rot) = listener.lookupTransform('ned', 'target', rospy.Time(0))
            
            msg_tf = PoseStamped()
            msg_tf.header = Header()
            msg_tf.header.frame_id = "ned"
            msg_tf.header.stamp = rospy.Time.now()

            msg_tf.pose.position.x = trans[0]
            msg_tf.pose.position.y = trans[1]
            msg_tf.pose.position.z = trans[2]

            msg_tf.pose.orientation.x = rot[0]
            msg_tf.pose.orientation.y = rot[1]
            msg_tf.pose.orientation.z = rot[2]
            msg_tf.pose.orientation.w = rot[3]
            
            pub_tar.publish(msg_tf)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print "can not get transfer!!!!"
        rate.sleep()




