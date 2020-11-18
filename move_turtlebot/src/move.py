#! /usr/bin/env python

# Created by Yamina Katariya and Suzanne Cuozzo

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry 
#from sensor_msgs.msg import BatteryState #not needed for simulated robots

#Dictionary of all robots in the swarm
turtlebot_dict = {
    "turtlebot1" : "tb3_0/",
    "turtlebot2" : "tb3_1/",
    "turtlebot3" : "tb3_2/",
    "turtlebot4" : "tb3_3/"
}

imu_xorientation = 0.0
imu_yorientation = 0.0
imu_zorientation = 0.0
imu_worientation = 0.0

def LaserScan_callback(msg):

    global partially_failed_laser
    low_range = 0.0
    high_range = 0.0

    #Check for laser scanner faults
    for i in range(len(msg.ranges)):
        if (msg.ranges[i] <= low_range) or (msg.ranges[i] >= high_range):
            partially_failed_laser = True
            break

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

    #Check for IMU faults
    if ((msg.angular_velocity.x or msg.angular_velocity.y or msg.angular_velocity.z) <= low_angular_vel) or ((msg.angular_velocity.x or msg.angular_velocity.y or msg.angular_velocity.z) <= high_angular_vel):
        partially_failed_imu = True

    if ((msg.linear_acceleration.x or msg.linear_acceleration.y or msg.linear_acceleration.z) <= low_linear_acc) or ((msg.linear_acceleration.x or msg.linear_acceleration.y or msg.linear_acceleration.z) >= high_linear_acc):
        partially_failed_imu = True

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

    #Check for odometer faults
    if ((msg.pose.pose.position.x or msg.pose.pose.position.y or msg.pose.pose.position.x) <= low_position) or ((msg.pose.pose.position.x or msg.pose.pose.position.y or msg.pose.pose.position.z) >= high_position):
        partially_failed_odom = True

    if ((msg.twist.twist.linear.x or msg.twist.twist.linear.y or msg.twist.twist.linear.z) <= low_twist) or ((msg.twist.twist.linear.x or msg.twist.twist.linear.y or msg.twist.twist.linear.z) >= high_twist):
        partially_failed_odom = True

    if ((msg.twist.twist.angular.x or msg.twist.twist.angular.y or msg.twist.twist.angular.z) <= low_twist) or ((msg.twist.twist.angular.x or msg.twist.twist.angular.y or msg.twist.twist.angular.z) >= high_twist):
        partially_failed_odom = True

    if (imu_xorientation != msg.pose.pose.orientation.x) or (imu_yorientation != msg.pose.pose.orientation.y) or (imu_zorientation != msg.pose.pose.orientation.z) or (imu_worientation != msg.pose.pose.orientation.w):
        partially_failed_odom = True

def err_tb(tb3_name):
    
    global partially_failed_laser
    global partially_failed_imu
    global partially_failed_odom

    partially_failed_laser = False
    partially_failed_imu = False
    partially_failed_odom = False

    #Initialize turtlebot3 node
    rospy.init_node('current_tb')
    r = rospy.Rate(10)

    count = 1 #Variable to ensure each robot is checked maxNumTries times in a row before moving onto the next one
    maxNumTries = 4 #Maximum number of times to check each robot before moving on (if it's not faulty)

    #Count codes to ensure partial faults are double checked
    laserCheckAgainCode = -2 
    imuCheckAgainCode = -3
    odomCheckAgainCode = -4

    while True:
        print("")
        print(tb3_name)
        #FOR TESTING: Faulty robot (the one running error injectors) is turtlebot1. If change which robot is faulty, change line below.
        if tb3_name == "turtlebot1": #Change this line when switching faulty turtlebot
            #Subscribe to laserscan, imu, and odom
            LaserScan_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'laser_err_inj', LaserScan, LaserScan_callback)
            print("LaserScan fault: " + str(partially_failed_laser))
            if partially_failed_laser and count != (laserCheckAgainCode): #Found fault 1st time
                print('LaserScan fault found! Double checking...')
                count = laserCheckAgainCode
                continue
            if partially_failed_laser and count == (laserCheckAgainCode): #Found fault again after double checking
                print('LaserScan fault still found! ' + tb3_name + ' has partially failed')
                #Send msg here!!!
                count = maxNumTries + 1 #To exit loop
                break

            Imu_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'imu_err_inj', Imu, Imu_callback)
            print("IMU fault: " + str(partially_failed_imu))
            if partially_failed_imu and count != (imuCheckAgainCode): #Found fault 1st time
                print('IMU fault found! Double checking...')
                count = imuCheckAgainCode
                continue
            if partially_failed_imu and count == (imuCheckAgainCode): #Found fault again after double checking
                print('IMU fault still found! ' + tb3_name + ' has partially failed')
                #Send msg here!!!
                count = maxNumTries + 1 #To exit loop
                break

            Odom_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'odom_err_inj', Odometry, Odom_callback)
            print("Odom fault: " + str(partially_failed_odom))
            if partially_failed_odom and count != (odomCheckAgainCode): #Found fault 1st time 
                print('Odom fault found! Double checking...')
                count = odomCheckAgainCode
                continue
            if partially_failed_odom and count == (odomCheckAgainCode): #Found fault again after double checking
                print('Odom fault still found! ' + tb3_name + ' has partially failed')
                #Send msg here!!!
                count = maxNumTries + 1 #To exit loop
                break


            #Timeout (move onto next robot) after checking robot maxNumTries times and not finding a fault
            if count > maxNumTries:
                break;
            count = count + 1; 

            r.sleep()
        #All other robots in swarm
        else:
            #Subscribe to laserscan, imu, and odom
            LaserScan_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'scan', LaserScan, LaserScan_callback)
            print("LaserScan fault: " + str(partially_failed_laser))
            if partially_failed_laser and count != (laserCheckAgainCode): #Found fault 1st time
                print('LaserScan fault found! Double checking...')
                count = laserCheckAgainCode
                continue
            if partially_failed_laser and count == (laserCheckAgainCode): #Found fault again after double checking
                print('LaserScan fault still found! ' + tb3_name + ' has partially failed')
                #Send msg here!!!
                count = maxNumTries + 1 #To exit loop
                break

            Imu_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'imu', Imu, Imu_callback)
            print("IMU fault: " + str(partially_failed_imu))
            if partially_failed_imu and count != (imuCheckAgainCode): #Found fault 1st time 
                print('IMU fault found! Double checking...')
                count = imuCheckAgainCode
                continue
            if partially_failed_imu and count == (imuCheckAgainCode): #Found fault again after double checking
                print('IMU fault still found! ' + tb3_name + ' has partially failed')
                #Send msg here!!!
                count = maxNumTries + 1 #To exit loop
                break

            Odom_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'odom', Odometry, Odom_callback)
            print("Odom fault: " + str(partially_failed_odom))
            if partially_failed_odom and count != odomCheckAgainCode: #Found fault 1st time
                print('Odom fault found! Double checking...')
                count = odomCheckAgainCode
                continue
            if partially_failed_odom and count == (odomCheckAgainCode): #Found fault again after double checking
                print('Odom fault still found! ' + tb3_name + ' has partially failed')
                #Send msg here!!!
                count = maxNumTries + 1 #To exit loop
                break


            #Timeout (move onto next robot) after checking robot maxNumTries times and not finding a fault
            if count > maxNumTries:
                break
            count = count + 1;

            r.sleep()

#Booleans that show if there are faults in a robot / where they are. No fault = FALSE, fault = TRUE
partially_failed_laser = False
partially_failed_imu = False
partially_failed_odom = False

#Call algorithm on each robot sequentially
while True:
    tb3_0 = err_tb("turtlebot1")
    tb3_1 = err_tb("turtlebot2")
    tb3_2 = err_tb("turtlebot3")
    tb3_3 = err_tb("turtlebot4")
