#!/usr/bin/env python
from sslib import *
from uwbfilter import *
from load_uwbdata.msg import uwbdata
from kalmanfilter.msg import state
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import Imu

global q,a,r
a = array([0,0,0])
r = array([0,0,0])
q = array([0,0,0,1])

def imucallback(msg):
    global q,a,r
    q = array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    a = array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    r = array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

def estimation(msg):
    uwbdis = msg.distance
    #need transform to ned frame (lookup, frame_id)
    uwbanchor = array([msg.position.x, msg.position.y, msg.position.z]) 
    global q,a,r

    xe, _ = uwb.locate(xe, Q, 1.0/100, uwbdis, uwbanchor, q, a, r)

    x_msg = state()
    x_msg.state.header = Header()
    x_msg.state.header.frame_id = "ned"
    x_msg.state.header.stamp = rospy.Time.now()
    x_msg.state.pose.position = Point(xe[0], xe[1], xe[2])
    x_msg.state.pose.orientation = Quaternion(xe[3], xe[4], xe[5], xe[6])
    x_msg.velocity = Vector3(xe[7], xe[8], xe[9])
    x_msg.bias = xe[10]

    pub.publish(x_msg)
    

'''    
    print "Accuracy:", linalg.norm(xe[:,0:3]-p)
    print "std:", std(xe[:,0:3]-p)
    print "Time: ", timer.end()'''



if __name__ == '__main__':
    rospy.init_node('localization')
    rospy.Subscriber('/uwb_node/uwb_distance', uwbdata, estimation)
    rospy.Subscriber('/mavros/imu/data', Imu, imucallback)
    pub = rospy.Publisher('est_state', state, queue_size=0)	
    
    xe = zeros((1,11))[0]
    xe[2] = 0.27
    xe[6] = 1
    Q[ 0:3,  0:3] =  0.98*eye(3)
    Q[ 3:7,  3:7] =  0.01*eye(4)
    Q[ 7:9,  7:9] =  0.81*eye(2)
    Q[  9 ,   9 ] =  900
    Q[ 10 ,  10 ] =  0.000000001
    #1.28 0.81 400 num = 2
    #0.98 0.81,900 num =1
    #0.5 0.01 100 num = 10
    Q = Q*7
    uwb = UWBLocation(1.0/100) 
    uwb.setQ(Q)   
    
    timer = sstimer()
    timer.start()

    rospy.spin()
