#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

turtlebot_dict = {
    "turtlebot1" : "tb3_0",
    "turtlebot2" : "tb3_1",
    "turtlebot3" : "tb3_2",
    "turtlebot4" : "tb3_3"
}

inf = 0.0 #Should probably change this, the inf was just being annoying

#Variables for the values coming from the (real) scan topic
actual_seq = 0
actual_secs = 0
actual_nsecs = 0
actual_frameid = ""

actual_xorientation = 0.0
actual_yorientation = 0.0
actual_zorientation = 0.0
actual_worientation = 0.0
actual_orientationcovariance = [0.0] * 9

actual_xangularvelocity = 0.0
actual_yangularvelocity = 0.0
actual_zangularvelocity = 0.0
actual_angularvelocitycovariance = [0.0] * 9

actual_xlinearacceleration = 0.0
actual_ylinearacceleration = 0.0
actual_zlinearacceleration = 0.0
actual_linearaccelerationcovariance = [0.0] * 9

#Get real sensor values
def listener(msg):
    global actual_seq
    global actual_secs
    global actual_nsecs
    global actual_frameid

    global actual_xorientation
    global actual_yorientation
    global actual_zorientation
    global actual_worientation
    global actual_orientationcovariance

    global actual_xangularvelocity
    global actual_yangularvelocity
    global actual_zangularvelocity
    global actual_angularvelocitycovariance

    global actual_xlinearacceleration
    global actual_ylinearacceleration
    global actual_zlinearacceleration
    global actual_linearaccelerationcovariance
    

    actual_seq = msg.header.seq
    actual_secs = msg.header.stamp.secs
    actual_nsecs = msg.header.stamp.nsecs
    actual_frameid = msg.header.frame_id

    actual_xorientation = msg.orientation.x
    actual_yorientation = msg.orientation.y
    actual_zorientation = msg.orientation.z
    actual_worientation = msg.orientation.w
    for i in range(len(msg.orientation_covariance)):
        actual_orientationcovariance[i] = msg.orientation_covariance[i]

    actual_xangularvelocity = msg.angular_velocity.x
    actual_yangularvelocity = msg.angular_velocity.y
    actual_zangularvelocity = msg.angular_velocity.z
    for j in range(len(msg.angular_velocity_covariance)):
        actual_angularvelocitycovariance[j] = msg.angular_velocity_covariance[j]

    actual_xlinearacceleration = msg.linear_acceleration.x
    actual_ylinearacceleration = msg.linear_acceleration.y
    actual_zlinearacceleration = msg.linear_acceleration.z
    for k in range(len(msg.linear_acceleration_covariance)):
        actual_linearaccelerationcovariance[k] = msg.linear_acceleration_covariance[k]

#########################################
def imu_err_inj(tb3_name):
    #Create error-injected topic
    rospy.init_node('imu_err_inj')

    #Create new message
    imu_msg = Imu() 

    #Fill message with values
    imu_msg.header.seq = 0
    imu_msg.header.stamp.secs = 0
    imu_msg.header.stamp.nsecs = 0
    imu_msg.header.frame_id = ""

    imu_msg.orientation.x = 0.0
    imu_msg.orientation.y = 0.0
    imu_msg.orientation.z = 0.0
    imu_msg.orientation.w = 0.0
    imu_msg.orientation_covariance = [0.0] * 9

    imu_msg.angular_velocity.x = 0.0
    imu_msg.angular_velocity.y = 0.0
    imu_msg.angular_velocity.z = 0.0
    imu_msg.angular_velocity_covariance = [0.0] * 9

    imu_msg.linear_acceleration.x = 0.0
    imu_msg.linear_acceleration.y = 0.0
    imu_msg.linear_acceleration.z = 0.0
    imu_msg.linear_acceleration_covariance = [0.0] * 9


    #########################################
    rate = rospy.Rate(1)

    #Publish message into new topic
    while not rospy.is_shutdown(): 
        my_pub = rospy.Publisher(tb3_name + '/imu_err_inj', Imu, queue_size = 10) 
        my_sub = rospy.Subscriber(tb3_name + '/imu', Imu, listener)

        #########################################
        #INJECT ERRORS HERE
        imu_msg.header.seq = actual_seq
        imu_msg.header.stamp.secs = actual_secs
        imu_msg.header.stamp.nsecs = actual_nsecs
        imu_msg.header.frame_id = actual_frameid

        imu_msg.orientation.x = actual_xorientation * 0.0
        imu_msg.orientation.y = actual_yorientation * 0.0
        imu_msg.orientation.z = actual_zorientation * 0.0
        imu_msg.orientation.w = actual_worientation * 0.0
        for i in range(len(actual_orientationcovariance)):
            imu_msg.orientation_covariance[i] = actual_orientationcovariance[i] * 0.0

        imu_msg.angular_velocity.x = actual_xangularvelocity * 0.0
        imu_msg.angular_velocity.y = actual_yangularvelocity * 0.0
        imu_msg.angular_velocity.z = actual_xangularvelocity * 0.0
        for j in range(len(actual_angularvelocitycovariance)):
            imu_msg.angular_velocity_covariance[j] = actual_angularvelocitycovariance[j] * 0.0

        imu_msg.linear_acceleration.x = actual_xlinearacceleration * 0.0
        imu_msg.linear_acceleration.y = actual_ylinearacceleration * 0.0
        imu_msg.linear_acceleration.z = actual_zlinearacceleration * 0.0
        for k in range(len(actual_linearaccelerationcovariance)):
            imu_msg.linear_acceleration_covariance[k] = actual_linearaccelerationcovariance[k] * 0.0

        #########################################
            
        my_pub.publish(imu_msg)
        rate.sleep()
        
    rospy.spin()
        
if __name__ == '__main__':
    tb3_0 = imu_err_inj("turtlebot1")
    #tb3_1 = imu_err_inj("turtlebot2")
    #tb3_2 = imu_err_inj("turtlebot3")
    #tb3_3 = imu_err_inj("turtlebot4")

