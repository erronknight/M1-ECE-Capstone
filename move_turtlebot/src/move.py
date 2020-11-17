#! /usr/bin/env python

# Created by Yamina Katariya and Suzanne Cuozzo

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

#from sensor_msgs.msg import BatteryState

turtlebot_dict = {
    "turtlebot1" : "tb3_0/",
    "turtlebot2" : "tb3_1/",
    "turtlebot3" : "tb3_2/",
    "turtlebot4" : "tb3_3/"
}

#partially_failed_laser = False
#partially_failed_imu = False
#partially_failed_odom = False

imu_xorientation = 0.0
imu_yorientation = 0.0
imu_zorientation = 0.0
imu_worientation = 0.0

def LaserScan_callback(msg):

    global partially_failed_laser

    low_range = 0.0
    high_range = 0.0

    for i in range(len(msg.ranges)):
        if (msg.ranges[i] <= low_range) or (msg.ranges[i] >= high_range):
            partially_failed_laser = True

    #print("LaserScan fault: " + str(partially_failed_laser))

def Imu_callback(msg):

    global partially_failed_imu
    global imu_xorientation
    global imu_yorientation  
    global imu_zorientation
    global imu_worientation

    imu_xorientation = msg.orientation.x
    imu_yorientation = msg.orientation.y
    imu_zorientation = msg.orientation.z
    imu_worientation = msg.orientation.w

    low_angular_vel = 0.0
    high_angular_vel = 0.0
    low_linear_acc = 0.0
    high_linear_acc = 0.0

    if ((msg.angular_velocity.x or msg.angular_velocity.y or msg.angular_velocity.z) <= low_angular_vel) or ((msg.angular_velocity.x or msg.angular_velocity.y or msg.angular_velocity.z) <= high_angular_vel):
        partially_failed_imu = True

    if ((msg.linear_acceleration.x or msg.linear_acceleration.y or msg.linear_acceleration.z) <= low_linear_acc) or ((msg.linear_acceleration.x or msg.linear_acceleration.y or msg.linear_acceleration.z) >= high_linear_acc):
        partially_failed_imu = True

    #print("HELLO" + str(partially_failed_imu))

def Odom_callback(msg):
    
    global partially_failed_odom
    global imu_xorientation
    global imu_yorientation  
    global imu_zorientation
    global imu_worientation

    low_position = 0.0
    high_position = 0.0
    low_twist = 0.0
    high_twist = 0.0

    if ((msg.pose.pose.position.x or msg.pose.pose.position.y or msg.pose.pose.position.x) <= low_position) or ((msg.pose.pose.position.x or msg.pose.pose.position.y or msg.pose.pose.position.z) >= high_position):
        partially_failed_odom = True

    if ((msg.twist.twist.linear.x or msg.twist.twist.linear.y or msg.twist.twist.linear.z) <= low_twist) or ((msg.twist.twist.linear.x or msg.twist.twist.linear.y or msg.twist.twist.linear.z) >= high_twist):
        partially_failed_odom = True

    if ((msg.twist.twist.angular.x or msg.twist.twist.angular.y or msg.twist.twist.angular.z) <= low_twist) or ((msg.twist.twist.angular.x or msg.twist.twist.angular.y or msg.twist.twist.angular.z) >= high_twist):
        partially_failed_odom = True

    if (imu_xorientation != msg.pose.pose.orientation.x) or (imu_yorientation != msg.pose.pose.orientation.y) or (imu_zorientation != msg.pose.pose.orientation.z) or (imu_worientation != msg.pose.pose.orientation.w):
        partially_failed_odom = True

    #print("Odom fault: " + str(partially_failed_odom))

def err_tb(tb3_name):
    
    global partially_failed_laser
    global partially_failed_imu
    global partially_failed_odom

    partially_failed_laser = False
    partially_failed_imu = False
    partially_failed_odom = False

    # initialize turtlebot3 node
    rospy.init_node('current_tb')
    r = rospy.Rate(10)
    
    while True:
        print(tb3_name)
        if tb3_name == "turtlebot1":
            # subscribe to laserscan, imu, and odom
            LaserScan_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'laser_err_inj', LaserScan, LaserScan_callback)
            print("LaserScan fault: " + str(partially_failed_laser))
            if partially_failed_laser:
                break
            Imu_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'imu_err_inj', Imu, Imu_callback)
            print("Imu fault: " + str(partially_failed_imu))
            if partially_failed_imu:
                break
            Odom_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'odom_err_inj', Odometry, Odom_callback)
            print("Odom fault: " + str(partially_failed_odom))
            if partially_failed_odom:
                break
            r.sleep()
        else:
            # subscribe to laserscan, imu, and odom
            LaserScan_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'scan', LaserScan, LaserScan_callback)
            print("LaserScan fault: " + str(partially_failed_laser))
            if partially_failed_laser:
                break
            Imu_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'imu', Imu, Imu_callback)
            print("Imu fault: " + str(partially_failed_imu))
            if partially_failed_imu:
                break
            Odom_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'odom', Odometry, Odom_callback)
            print("Odom fault: " + str(partially_failed_odom))
            if partially_failed_odom:
                break
            r.sleep()


partially_failed_laser = False
partially_failed_imu = False
partially_failed_odom = False

while True:
    tb3_0 = err_tb("turtlebot1")
    tb3_1 = err_tb("turtlebot2")
    tb3_2 = err_tb("turtlebot3")
    tb3_3 = err_tb("turtlebot4")
