#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

turtlebot_dict = {
    "turtlebot1" : "tb3_0",
    "turtlebot2" : "tb3_1",
    "turtlebot3" : "tb3_2",
    "turtlebot4" : "tb3_3"
}


#Variables for the values coming from the (real) scan topic
actual_seq = 0
actual_secs = 0
actual_nsecs = 0
actual_frameid = ""

actual_childframeid = ""

actual_xposition = 0.0
actual_yposition = 0.0
actual_zposition = 0.0
actual_xorientation = 0.0
actual_yorientation = 0.0
actual_zorientation = 0.0
actual_worientation = 0.0
actual_posecovariance = [0.0] * 36

actual_xlinear = 0.0
actual_ylinear = 0.0
actual_zlinear = 0.0
actual_xangular = 0.0
actual_yangular = 0.0
actual_zangular = 0.0
actual_twistcovariance = [0.0] * 36

#Get real sensor values
def listener(msg):
    global actual_seq
    global actual_secs
    global actual_nsecs
    global actual_frameid

    global actual_childframeid

    global actual_xposition
    global actual_yposition
    global actual_zposition
    global actual_xorientation
    global actual_yorientation
    global actual_zorientation
    global actual_worientation
    global actual_posecovariance

    global actual_xlinear
    global actual_ylinear
    global actual_zlinear
    global actual_xangular
    global actual_yangular
    global actual_zangular
    global actual_twistcovariance


    actual_seq = msg.header.seq
    actual_secs = msg.header.stamp.secs
    actual_nsecs = msg.header.stamp.nsecs
    actual_frameid = msg.header.frame_id

    actual_childframeid = msg.child_frame_id

    actual_xposition = msg.pose.pose.position.x
    actual_yposition = msg.pose.pose.position.y
    actual_zposition = msg.pose.pose.position.z
    actual_xorientation = msg.pose.pose.orientation.x
    actual_yorientation = msg.pose.pose.orientation.y
    actual_zorientation = msg.pose.pose.orientation.z
    actual_worientation = msg.pose.pose.orientation.w
    for i in range(len(msg.pose.covariance)):
        actual_posecovariance[i] = msg.pose.covariance[i]

    actual_xlinear = msg.twist.twist.linear.x
    actual_ylinear = msg.twist.twist.linear.y
    actual_zlinear = msg.twist.twist.linear.z
    for j in range(len(msg.twist.covariance)):
        actual_twistcovariance[j] = msg.twist.covariance[j]

def odom_err_inj(tb3_name):
    #Create error-injected topic
    rospy.init_node('odom_err_inj')

    #########################################
    #Create new message
    odom_msg = Odometry()


    #Fill message with values
    odom_msg.header.seq = 0
    odom_msg.header.stamp.secs = 0
    odom_msg.header.stamp.nsecs = 0
    odom_msg.header.frame_id = ""

    odom_msg.child_frame_id = ""

    odom_msg.pose.pose.position.x = 0.0
    odom_msg.pose.pose.position.y = 0.0
    odom_msg.pose.pose.position.z = 0.0
    odom_msg.pose.pose.orientation.x = 0.0
    odom_msg.pose.pose.orientation.y = 0.0
    odom_msg.pose.pose.orientation.z = 0.0
    odom_msg.pose.pose.orientation.w = 0.0
    odom_msg.pose.covariance = [0.0] * 36

    odom_msg.twist.twist.linear.x = 0.0
    odom_msg.twist.twist.linear.y = 0.0
    odom_msg.twist.twist.linear.z = 0.0
    odom_msg.twist.twist.angular.x = 0.0
    odom_msg.twist.twist.angular.y = 0.0
    odom_msg.twist.twist.angular.z = 0.0
    odom_msg.twist.covariance = [0.0] * 36

    #########################################
    rate = rospy.Rate(50)

    #Publish message into new topic
    while not rospy.is_shutdown(): 
        my_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + '/odom', Odometry, listener)
        my_pub = rospy.Publisher(turtlebot_dict[tb3_name] + '/odom_err_inj', Odometry, queue_size = 10) 

        #########################################
        #INJECT ERRORS HERE
        odom_msg.header.seq = actual_seq
        odom_msg.header.stamp.secs = actual_secs
        odom_msg.header.stamp.nsecs = actual_nsecs
        odom_msg.header.frame_id = actual_frameid

        odom_msg.child_frame_id = actual_childframeid

        odom_msg.pose.pose.position.x = actual_xposition 
        odom_msg.pose.pose.position.y = actual_yposition 
        odom_msg.pose.pose.position.z = actual_zposition 
        odom_msg.pose.pose.orientation.x = actual_xorientation
        odom_msg.pose.pose.orientation.y = actual_yorientation 
        odom_msg.pose.pose.orientation.z = actual_zorientation
        odom_msg.pose.pose.orientation.w = actual_worientation
        for i in range(len(actual_posecovariance)):
            odom_msg.pose.covariance[i] = actual_posecovariance[i] 

        odom_msg.twist.twist.linear.x = actual_xlinear
        odom_msg.twist.twist.linear.y = actual_ylinear 
        odom_msg.twist.twist.linear.z = actual_zlinear 
        odom_msg.twist.twist.angular.x = actual_xangular 
        odom_msg.twist.twist.angular.y = actual_yangular
        odom_msg.twist.twist.angular.z = actual_zangular
        for j in range(len(actual_twistcovariance)):
            odom_msg.twist.covariance[j] = actual_twistcovariance[j]

        #########################################
            
        my_pub.publish(odom_msg)
        rate.sleep()
    
    rospy.spin()
if __name__ == '__main__':
    tb3_0 = odom_err_inj("turtlebot1")
    #tb3_1 = odom_err_inj("turtlebot2")
    #tb3_2 = odom_err_inj("turtlebot3")
    #tb3_3 = odom_err_inj("turtlebot4")

