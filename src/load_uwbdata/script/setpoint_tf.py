#!/usr/bin/env python
 
import rospy
import thread
import threading
import time
import math
 
from geometry_msgs.msg import PoseStamped, Quaternion
from math import *
from mavros.srv import CommandBool
from mavros.utils import *
from std_msgs.msg import Header
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, PoseWithCovarianceStamped


class Setpoint:
 
    def __init__(self, hz = 10):

        self.rospy = rospy
        self.rospy.init_node('pose', anonymous=True)
        self.x, self.y, self.z, self.hz = 0.0, 0.0, 0.0, hz
        self.listener = tf.TransformListener()
 
        try:
            thread.start_new_thread( self.navigate, () )
        except:
            print "Error: Unable to start thread"
 
        self.done = False
        self.done_evt = threading.Event()
 
    def navigate(self):

        rate = self.rospy.Rate(self.hz)
        br = tf.TransformBroadcaster()
 
        while not self.rospy.is_shutdown():
            quaternion = quaternion_from_euler(0, 0, self.yaw)
            now = rospy.Time.now()
            br.sendTransform((self.x, self.y, self.z), quaternion, now, "target", "vicon")
            rate.sleep()

    def set(self, x, y, z, yaw):
        self.fixpoint_heading = False
        self.done = False
        self.x, self.y, self.z, self.yaw = x, y, z, radians(yaw)
        rate = rospy.Rate(self.hz)
        while not self.rospy.is_shutdown() and not self.done:
            self.reached()
            rate.sleep()

    def set_without_reached(self, x, y, z, yaw):
        self.fixpoint_heading = False
        self.done = False
        self.x, self.y, self.z, self.yaw = x, y, z, radians(yaw)

    def set_fixpoint_heading(self, x, y, z, tx, ty, tz):
        '''(tx ty tz) is the heading in vicon frame'''
        self.fixpoint_heading = True
        self.done = False
        self.x,  self.y,  self.z  = x,  y,  z
        self.tx, self.ty, self.tz = tx, ty, tz
        self.yaw = 0
        rate = self.rospy.Rate(self.hz)
        while not self.rospy.is_shutdown() and not self.done:
            self.reached()
            rate.sleep()

    def set_without_reached_fixpoint_heading(self, x, y, z, tx, ty, tz):
        self.fixpoint_heading = True
        self.done = False
        self.x,  self.y,  self.z  = x,  y,  z
        self.tx, self.ty, self.tz = tx, ty, tz
        self.yaw = 0
    def check_reached(self):
        '''To check arrived after call ---> set_with_arrived'''
        self.reached()
        return self.done
 
    def reached(self):
        try:
            (position, rot) = self.listener.lookupTransform('vicon', 'robot_base', rospy.Time(0))
            if self.fixpoint_heading:
                self.yaw = atan2(self.ty - position[1], self.tx - position[0])
            euler = euler_from_quaternion(rot)
            if abs(position[0] - self.x) < 0.3 and abs(position[1] - self.y) < 0.3 and abs(position[2] - self.z) < 0.3 and abs(euler[2]-self.yaw) < 3.0/180*math.pi:
                self.done = True
        except:
            print "can not get  (vicon ---> robot_base)"
        self.done_evt.set()

def setpoint_demo():

    setpoint = Setpoint()
    setpoint.set(    0,     0, 1.07, 90)
    setpoint.set(-1.50,  1.45, 1.07, 0)
    setpoint.set( 1.52,  1.49, 1.04, -90)
    setpoint.set( 1.55, -1.47, 1.05, 180)
    setpoint.set(-1.47, -1.47, 1.08, 90)
    setpoint.set(    0,     0, 1.08, 90)
    setpoint.set(    0,     0, -1.0, 90)
    print "Finished setpoint demo..." 

def setpoint_without_reached_demo():

    setpoint = Setpoint()
    setpoint.set_without_reached(    0,     0, 1.07, 90)

    while not self.rospy.is_shutdown() and not setpoint.check_reached():
        #you can do your own things here
        rate.sleep()
    print "Finished setpoint with reached demo..." 

def setpoint_fix_heading_demo():

    setpoint = Setpoint()
    setpoint.set_fixpoint_heading(    0,     0, 1.07, 0, 0.5, 0)
    setpoint.set_fixpoint_heading(-1.50,  1.45, 1.07, 0, 0.5, 0)
    setpoint.set_fixpoint_heading( 1.52,  1.49, 1.04, 0, 0.5, 0)
    setpoint.set_fixpoint_heading( 1.55, -1.47, 1.05, 0, 0.5, 0)
    setpoint.set_fixpoint_heading(-1.47, -1.47, 1.08, 0, 0.5, 0)
    setpoint.set_fixpoint_heading(    0,     0, 1.08, 0, 0.5, 0)
    setpoint.set_fixpoint_heading(    0,     0, -1.0, 0, 0.5, 0)
    print "Finished setpoint demo..." 

if __name__ == '__main__':
    #setpoint_demo()
    setpoint_fix_heading_demo()
    #please refer to setpoint_[*]demo to use this class


