#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

turtlebot_dict = {
    "turtlebot1" : "tb3_0/",
    "turtlebot2" : "tb3_1/",
    "turtlebot3" : "tb3_2/",
    "turtlebot4" : "tb3_3/"	
}

inf = 0.0 #Should probably change this, the inf was just being annoying

#Variables for the values coming from the (real) scan topic
actual_seq = 0
actual_secs = 0
actual_nsecs = 0
actual_frameid = ""

actual_anglemin = 0.0
actual_anglemax = 0.0
actual_angleincrement = 0.0
actual_timeincrement = 0.0
actual_scantime = 0.0
actual_rangemin = 0.0
actual_rangemax = 0.0
actual_ranges = [0.0] * 360
actual_intensities = [0.0] * 360

#Get real sensor values
def listener(msg):
    global actual_seq
    global actual_secs
    global actual_nsecs
    global actual_frameid

    global actual_anglemin
    global actual_anglemax
    global actual_angelincrement
    global actual_timeincrement
    global actual_scantime
    global actual_rangemin
    global actual_rangemax

    global actual_ranges
    global actual_intensities

    actual_seq = msg.header.seq
    actual_secs = msg.header.stamp.secs
    actual_nsecs = msg.header.stamp.nsecs
    actual_frameid = msg.header.frame_id

    actual_anglemin = msg.angle_min
    actual_anglemax = msg.angle_max
    actual_angleincrement = msg.angle_increment
    actual_timeincrement = msg.time_increment
    actual_scantime = msg.scan_time
    actual_rangemin = msg.range_min
    actual_rangemax = msg.range_max

    for i in range(len(msg.ranges)):
        actual_ranges = msg.ranges
    for j in range(len(msg.intensities)):
        actual_intensities = msg.intensities

def laserscan_err_inj(tb3_name):
    #Create error-injected topic
    rospy.init_node('laser_err_inj')

    #########################################
    #Create new message
    laserscan_msg = LaserScan() 

    #Fill message with values
    laserscan_msg.header.seq = 0 
    laserscan_msg.header.stamp.secs = 0
    laserscan_msg.header.stamp.nsecs = 0
    laserscan_msg.header.frame_id = ""

    laserscan_msg.angle_min = 0.0
    laserscan_msg.angle_max = 0.0
    laserscan_msg.angle_increment = 0.0
    laserscan_msg.time_increment = 0.0
    laserscan_msg.scan_time = 0.0
    laserscan_msg.range_min = 0.0
    laserscan_msg.range_max = 0.0

    laserscan_msg.ranges = [0.0] * 360
    laserscan_msg.intensities = [0.0] * 360

    #########################################

    rate = rospy.Rate(1)

    #Publish message into new topic
    while not rospy.is_shutdown(): 
        my_pub = rospy.Publisher(tb3_name+'laser_err_inj', LaserScan, queue_size = 10) 
        my_sub = rospy.Subscriber(tb3_name+'scan', LaserScan, listener)

        #########################################
        #INJECT ERRORS HERE
        laserscan_msg.header.seq = actual_seq 
        laserscan_msg.header.stamp.secs = actual_secs
        laserscan_msg.header.stamp.nsecs = actual_nsecs
        laserscan_msg.header.frame_id = actual_frameid

        laserscan_msg.angle_min = actual_anglemin
        laserscan_msg.angle_max = actual_anglemax
        laserscan_msg.angle_increment = actual_angleincrement
        laserscan_msg.time_increment = actual_timeincrement
        laserscan_msg.scan_time = actual_scantime
        laserscan_msg.range_min = actual_rangemin
        laserscan_msg.range_max = actual_rangemax

        for i in range(len(actual_ranges)):
            laserscan_msg.ranges[i] = actual_ranges[i] * 0 #inject error here (i just made everything = 0 because it's easy to see when testing)

        for j in range(len(actual_intensities)):
            laserscan_msg.intensities[i] = actual_intensities[i] * 0 #inject error here (i just made everything = 0 because it's easy to see when testing)
        #########################################
            
        my_pub.publish(laserscan_msg)
        rate.sleep()
    
    rospy.spin()

if __name__ == '__main__':
    tb3_0 = laserscan_err_inj("turtlebot1")
    tb3_1 = laserscan_err_inj("turtlebot2")
    tb3_2 = laserscan_err_inj("turtlebot3")
    tb3_3 = laserscan_err_inj("turtlebot4")

 

