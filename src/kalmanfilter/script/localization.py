#!/usr/bin/env python
from sslib import *
from uwbfilter import *
from load_uwbdata.msg import uwbdata
from kalmanfilter.msg import state
from geometry_msgs.msg import Point, Quaternion, Vector3, PointStamped, PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import tf
import time
global q,a,r,xe
a = array([0,0,0])
r = array([0,0,0])
q = array([0,0,0,1])
xe = zeros((1,11))[0]
xe[2] = 0.27
xe[6] = 1

imuReceived = False

def imucallback(msg):
    global imuReceived
    imuReceived = True
    global q,a,r
    q = array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    a = array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    r = array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

def estimation(msg):
    uwbdis = msg.distance

    point = PointStamped()
    point.header = Header()
    point.header.frame_id = "vicon"
    point.header.stamp = rospy.Time.now()
    point.point = msg.position

 
    listener.waitForTransform("ned", "vicon", rospy.Time.now(), rospy.Duration(0.05))
    point_tf = listener.transformPoint("ned", point)
    uwbanchor = array([point_tf.point.x, point_tf.point.y, point_tf.point.z])

#    br = tf.TransformBroadcaster()
#    now =  rospy.Time.now()
#    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
#    br.sendTransform((msg.position.x, msg.position.y, msg.position.z), quaternion, now, "uwbanchor", "vicon")
#    listener = tf.TransformListener()
#    (trans, rot) = listener.lookupTransform('ned', 'uwbanchor', rospy.Time(0))
#    uwbanchor = array([trans[0], trans[1], trans[2]]) 
    
    global q,a,r,xe
    if (imuReceived):
        xe, _ = uwb.locate(xe, Q, 2.0/100, uwbdis, uwbanchor, q, a, r)
#    print xe

#    x_msg = state()
#    x_msg.state.header = Header()
#    x_msg.state.header.frame_id = "ned"
#    x_msg.state.header.stamp = rospy.Time.now()
#    x_msg.state.pose.position = Point(xe[0], xe[1], xe[2])
#    x_msg.state.pose.orientation = Quaternion(xe[3], xe[4], xe[5], xe[6])
#    x_msg.velocity = Vector3(xe[7], xe[8], xe[9])
#    x_msg.bias = xe[10]

        x_msg = PoseStamped()
        x_msg.header = Header()
        x_msg.header.frame_id = "ned"
        x_msg.header.stamp = rospy.Time.now()
        x_msg.pose.position = Point(xe[0], xe[1], xe[2])
        x_msg.pose.orientation = Quaternion(xe[3], xe[4], xe[5], xe[6])
        pub.publish(x_msg)
    
        print "estimated position"
        print "[", "%.3f %.3f %.3f" % (xe[0], xe[1], xe[2]), "]"

    
#    print "Accuracy:", linalg.norm(xe[:,0:3]-p)
#    print "std:", std(xe[:,0:3]-p)
#    print "Time: ", timer.end()

if __name__ == '__main__':

    Q[ 0:3,  0:3] =  0.98*eye(3)
    Q[ 3:7,  3:7] =  0.01*eye(4)
    Q[ 7:9,  7:9] =  0.81*eye(2)
    Q[  9 ,   9 ] =  900
    Q[ 10 ,  10 ] =  0.000000001
    Q = Q*7
    uwb = UWBLocation(2.0/100) 
    uwb.setQ(Q)   

    rospy.init_node('localization')

#    rospy.Subscriber('/mavros/imu/data', Imu, imucallback)
    rospy.Subscriber('/imu', Imu, imucallback)
    pub = rospy.Publisher('est_state', PoseStamped, queue_size=0)	
    listener = tf.TransformListener()
    rate = rospy.Rate(1)
    for i in xrange(5):
        rate.sleep()
    rospy.Subscriber('/uwb_node/uwb_distance', uwbdata, estimation)
    rospy.spin()


