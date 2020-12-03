#! /usr/bin/env python

# Created by Yamina Katariya and Suzanne Cuozzo

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import time 
from id_msg.msg import CustomId

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
    infinity = float('inf')
    partially_failed_laser = False #Should be false to start

    low_range = .12-0.015
    high_range = 3.5+0.175

    #Check for laser scanner faults
    for i in range(len(msg.ranges)):
        if (msg.ranges[i] <= low_range) or (msg.ranges[i] >= high_range and msg.ranges[i] != infinity):
            partially_failed_laser = True
            LaserScan_sub.unregister()
            return

    LaserScan_sub.unregister()

def Imu_callback(msg):
    global partially_failed_imu
    global imu_xorientation
    global imu_yorientation  
    global imu_zorientation
    global imu_worientation

    partially_failed_imu = False #Should be false to start
    imu_xorientation = msg.orientation.x
    imu_yorientation = msg.orientation.y
    imu_zorientation = msg.orientation.z
    imu_worientation = msg.orientation.w

    low_angular_vel = -2.84-0.05
    high_angular_vel = 2.84+0.05
    low_linear_acc = 0.0-0.05
    high_linear_acc = 2.5+0.05

    if ((msg.angular_velocity.x <= low_angular_vel) or (msg.angular_velocity.y <= low_angular_vel) or (msg.angular_velocity.z <= low_angular_vel) or (msg.angular_velocity.x >= high_angular_vel) or (msg.angular_velocity.y >= high_angular_vel) or (msg.angular_velocity.z >= high_angular_vel) or (msg.linear_acceleration.x <= low_linear_acc) or (msg.linear_acceleration.y <= low_linear_acc) or (msg.linear_acceleration.z <= low_linear_acc) or (msg.linear_acceleration.x >= high_linear_acc) or (msg.linear_acceleration.y >= high_linear_acc) or (msg.linear_acceleration.z >= high_linear_acc)):
        partially_failed_imu = True
        Imu_sub.unregister()
        return

    Imu_sub.unregister()

def Odom_callback(msg):
   
    global partially_failed_odom
    global imu_xorientation
    global imu_yorientation  
    global imu_zorientation
    global imu_worientation

    partially_failed_odom = False #Should be false to start

    #changes with respect to simulated area
    low_position = -11 
    high_position = 11

    low_twist = -0.22-0.05
    high_twist = 0.22+0.05

    #Check for odometer faults
    if (msg.pose.pose.position.x <= low_position) or (msg.pose.pose.position.y <= low_position) or (msg.pose.pose.position.z <= low_position) or (msg.pose.pose.position.x >= high_position) or (msg.pose.pose.position.y >= high_position) or (msg.pose.pose.position.z >= high_position):
        partially_failed_odom = True
        Odom_sub.unregister()
        return

    if (msg.twist.twist.linear.x <= low_twist) or (msg.twist.twist.linear.y <= low_twist) or (msg.twist.twist.linear.z <= low_twist) or (msg.twist.twist.linear.x >= high_twist) or (msg.twist.twist.linear.y >= high_twist) or (msg.twist.twist.linear.z >= high_twist):
        partially_failed_odom = True
        Odom_sub.unregister()
        return

    if (msg.twist.twist.angular.x <= low_twist) or (msg.twist.twist.angular.y <= low_twist) or (msg.twist.twist.angular.z <= low_twist) or (msg.twist.twist.angular.x >= high_twist) or (msg.twist.twist.angular.y >= high_twist) or (msg.twist.twist.angular.z >= high_twist):
        partially_failed_odom = True
        Odom_sub.unregister()
        return
    
    Odom_sub.unregister()

#Publish message to tell rest of the system the turtlebot's status (faulty or not)
def publish_custom_msg(tb3_name, isFaulty):
    msg_pub = rospy.Publisher('/id_msg', CustomId, queue_size=1)
    rate = rospy.Rate(25) #Should match rate in err_tb (i think)

    turtlebot_name = turtlebot_dict[tb3_name]

    message = CustomId()
    message.id = turtlebot_name[-2] #Get ID # of turtlebot
    message.name = tb3_name
    message.fail_flag = str(isFaulty)

    msg_pub.publish(message)
    rate.sleep()

    #msg_pub.unregister()


def err_tb(tb3_name):
   
    global partially_failed_laser
    global partially_failed_imu
    global partially_failed_odom

    global LaserScan_sub
    global Imu_sub
    global Odom_sub

    partially_failed_laser = False
    partially_failed_imu = False
    partially_failed_odom = False

    #Initialize turtlebot3 node
    rospy.init_node('current_tb')
    r = rospy.Rate(50) #If this is determined to be too fast, 25 should work

    count = 1 #Variable to ensure each robot is checked maxNumTries times in a row before moving onto the next one
    maxNumTries = 3 #Maximum number of times to check each robot before moving on (if it's not faulty)

    #Count codes to ensure partial faults are double checked
    laserCheckAgainCode = -2
    imuCheckAgainCode = -3
    odomCheckAgainCode = -4

    while True:
        print("")
        print(tb3_name)
        #FOR TESTING: Faulty robot (the one running error injectors) is turtlebot1. If change which robot is faulty, change line below.
	#If not running error injectors, change line below to ignore "if". So change it to: if False: 
        if tb3_name == "turtlebot1": 
            #Subscribe to laserscan, imu, and odom
            LaserScan_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'laser_err_inj', LaserScan, LaserScan_callback)
            #LaserScan_sub.shutdown()
            r.sleep()
            print("LaserScan fault: " + str(partially_failed_laser))
            if partially_failed_laser and count != (laserCheckAgainCode): #Found fault 1st time
                print('LaserScan fault found! Double checking...')
                count = laserCheckAgainCode
                partially_failed_laser = False
                continue
            if partially_failed_laser and count == (laserCheckAgainCode): #Found fault again after double checking
                print('LaserScan fault still found! ' + tb3_name + ' has partially failed')
                publish_custom_msg(tb3_name, True) #Update status to faulty
                count = maxNumTries + 1 #To exit loop
                partially_failed_laser = False
                break
            elif (not partially_failed_laser) and count == (laserCheckAgainCode): #No fault after double checking
                print('No LaserScan fault found in ' + tb3_name + '. Communication error likely.')
                count = maxNumTries + 1
                partially_failed_laser = False
                break
            elif (not partially_failed_laser) and count == (laserCheckAgainCode): #No fault after double checking
                print('No LaserScan fault found in ' + tb3_name + '. Communication error likely.')
                count = maxNumTries + 1
                partially_failed_laser = False
                publish_custom_msg(tb3_name, False) #Update status to not faulty
                break

            publish_custom_msg(tb3_name, False) #Update status to not faulty

            Imu_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'imu_err_inj', Imu, Imu_callback)
            #rospy.wait_for_message('imu_err_inj', Imu)
            r.sleep()
            print("IMU fault: " + str(partially_failed_imu))
            if partially_failed_imu and count != (imuCheckAgainCode): #Found fault 1st time
                print('IMU fault found! Double checking...')
                count = imuCheckAgainCode
                partially_failed_imu = False
                continue
            if partially_failed_imu and count == (imuCheckAgainCode): #Found fault again after double checking
                print('IMU fault still found! ' + tb3_name + ' has partially failed')
                publish_custom_msg(tb3_name, True) #Update status to faulty
                count = maxNumTries + 1 #To exit loop
                partially_failed_imu = False
                break
            elif (not partially_failed_imu) and count == (imuCheckAgainCode): #No fault after double checking
                print('No IMU fault found in ' + tb3_name + '. Communication error likely.')
                count = maxNumTries + 1
                partially_failed_imu = False
                break
            elif (not partially_failed_imu) and count == (imuCheckAgainCode): #No fault after double checking
                print('No IMU fault found in ' + tb3_name + '. Communication error likely.')
                count = maxNumTries + 1
                partially_failed_imu = False
                publish_custom_msg(tb3_name, False) #Update status to not faulty
                break
    
            publish_custom_msg(tb3_name, False) #Update status to not faulty

            Odom_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'odom_err_inj', Odometry, Odom_callback)
            r.sleep()
            print("Odom fault: " + str(partially_failed_odom))
            if partially_failed_odom and count != (odomCheckAgainCode): #Found fault 1st time
                print('Odom fault found! Double checking...')
                count = odomCheckAgainCode
                continue
            if partially_failed_odom and count == (odomCheckAgainCode): #Found fault again after double checking
                print('Odom fault still found! ' + tb3_name + ' has partially failed')
                publish_custom_msg(tb3_name, True) #Update status to faulty
                count = maxNumTries + 1 #To exit loop
                partially_failed_odom = False
                break
            elif (not partially_failed_odom) and count == (odomCheckAgainCode): #No fault after double checking
                print('No odom fault found in ' + tb3_name + '. Communication error likely.')
                count = maxNumTries + 1
                publish_custom_msg(tb3_name, False) #Update status to not faulty
                partially_failed_odom = False
                break

            publish_custom_msg(tb3_name, False) #Update status to not faulty

            #Timeout (move onto next robot) after checking robot maxNumTries times and not finding a fault
            if count > maxNumTries:
                break;
            count = count + 1;

            r.sleep()
        #All other robots in swarm
        else:
            #Subscribe to laserscan, imu, and odom
            LaserScan_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'scan', LaserScan, LaserScan_callback)
            r.sleep()
            print("LaserScan fault: " + str(partially_failed_laser))
            if partially_failed_laser and count != (laserCheckAgainCode): #Found fault 1st time
                print('LaserScan fault found! Double checking...')
                count = laserCheckAgainCode
                partially_failed_laser = False
                continue
            if partially_failed_laser and count == (laserCheckAgainCode): #Found fault again after double checking
                print('LaserScan fault still found! ' + tb3_name + ' has partially failed')
                publish_custom_msg(tb3_name, True) #Update status to faulty
                count = maxNumTries + 1 #To exit loop
                partially_failed_laser = False
                break
            elif (not partially_failed_laser) and count == (laserCheckAgainCode): #No fault after double checking
                print('No LaserScan fault found in ' + tb3_name + '. Communication error likely.')
                count = maxNumTries + 1
                partially_failed_laser = False
                break
            elif (not partially_failed_laser) and count == (laserCheckAgainCode): #No fault after double checking
                print('No LaserScan fault found in ' + tb3_name + '. Communication error likely.')
                count = maxNumTries + 1
                publish_custom_msg(tb3_name, False) #Update status to not faulty
                partially_failed_laser = False
                break
            
            publish_custom_msg(tb3_name, False) #Update status to not faulty

            Imu_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'imu', Imu, Imu_callback)
            r.sleep()
            print("IMU fault: " + str(partially_failed_imu))
            if partially_failed_imu and count != (imuCheckAgainCode): #Found fault 1st time
                print('IMU fault found! Double checking...')
                count = imuCheckAgainCode
                partially_failed_imu = False
                continue
            if partially_failed_imu and count == (imuCheckAgainCode): #Found fault again after double checking
                print('IMU fault still found! ' + tb3_name + ' has partially failed')
                publish_custom_msg(tb3_name, True) #Update status to faulty
                count = maxNumTries + 1 #To exit loop
                partially_failed_imu = False
                break
            elif (not partially_failed_imu) and count == (imuCheckAgainCode): #No fault after double checking
                print('No IMU fault found in ' + tb3_name + '. Communication error likely.')
                count = maxNumTries + 1
                partially_failed_imu = False
                publish_custom_msg(tb3_name, False) #Update status to not faulty
                break

            publish_custom_msg(tb3_name, False) #Update status to not faulty

            Odom_sub = rospy.Subscriber(turtlebot_dict[tb3_name] + 'odom', Odometry, Odom_callback)
            r.sleep()
            print("Odom fault: " + str(partially_failed_odom))
            if partially_failed_odom and count != odomCheckAgainCode: #Found fault 1st time
                print('Odom fault found! Double checking...')
                count = odomCheckAgainCode
                partially_failed_odom = False
                continue
            if partially_failed_odom and count == (odomCheckAgainCode): #Found fault again after double checking
                print('Odom fault still found! ' + tb3_name + ' has partially failed')
                publish_custom_msg(tb3_name, True) #Update status to faulty
                count = maxNumTries + 1 #To exit loop
                partially_failed_odom = False
                break
            elif (not partially_failed_odom) and count == (odomCheckAgainCode): #No fault after double checking
                print('No odom fault found in ' + tb3_name + '. Communication error likely.')
                count = maxNumTries + 1
                partially_failed_odom = False
                publish_custom_msg(tb3_name, False) #Update status to not faulty
                break

            publish_custom_msg(tb3_name, False) #Update status to not faulty

            #Timeout (move onto next robot) after checking robot maxNumTries times and not finding a fault
            if count >= maxNumTries:
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
