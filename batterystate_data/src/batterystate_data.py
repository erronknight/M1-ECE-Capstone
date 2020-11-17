#! /usr/bin/env python

# THIS IS NOT FINISHED

import rospy
from sensor_msgs.msg import BatteryState

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

actual_voltage = 0.0
actual_current = 0.0
actual_charge = 0.0
actual_capacity = 0.0
actual_designcapacity = 0.0
actual_percentage = 0.0
actual_powersupplystatus = 0
actual_powersupplyhealth = 0
actual_powersupplytechnology = 0
actual_present = 1
actual_cellvoltage = []
actual_location = ""
actual_serialnumber = ""

#Get real sensor values
def listener(msg):
    global actual_seq
    global actual_secs
    global actual_nsecs
    global actual_frameid

    global actual_voltage
    global actual_current
    global actual_charge
    global actual_capacity
    global actual_designcpacity
    global actual_percentage
    global actual_powersupplystatus
    global actual_powersupplyhealth
    global actual_powersupplytechnology
    global actual_present
    global actual_cellvoltage

    global actual_location
    global actual_serialnumber

    actual_seq = msg.header.seq
    actual_secs = msg.header.stamp.secs
    actual_nsecs = msg.header.stamp.nsecs
    actual_frameid = msg.header.frame_id

    actual_voltage = msg.voltage
    actual_current = msg.current
    actual_charge = msg.charge
    actual_capacity = msg.capacity
    actual_designcapacity = msg.design_capacity
    actual_percentage = msg.percentage
    actual_powersupplystatus = msg.power_supply_status
    actual_powersupplyhealth = msg.power_supply_health
    actual_powersupplytechnology = msg.power_supply_technology
    actual_present = msg.present
    actual_cellvoltage = msg.cell_voltage
    actual_location = msg.location
    actual_serial_number = msg.serial_number

def bs_err_inj(tb3_name):
    #Create error-injected topic
    rospy.init_node('batterystate_err_inj')

    #########################################
    #Create new message
    batterystate_msg = BatteryState() 

    #Fill message with values
    batterystate_msg.header.seq = 0 
    batterystate_msg.header.stamp.secs = 0
    batterystate_msg.header.stamp.nsecs = 0
    batterystate_msg.header.frame_id = ""

    batterystate_msg.voltage = 0.0
    batterystate_msg.current = 0.0
    batterystate_msg.charge = 0.0
    batterystate_msg.capacity = 0.0
    batterystate_msg.design_capacity = 0.0
    batterystate_msg.percentage = 0.0
    batterystate_msg.power_supply_status = 0
    batterystate_msg.power_supply_health = 0
    batterystate_msg.power_supply_technology = 0
    batterystate_msg.bool = 1

    batterystate_msg.cell_voltage = []

    batterystate_msg.location = ""
    batterystate_msg.serial_number = ""

    #########################################

    rate = rospy.Rate(1)

    #Publish message into new topic
    while not rospy.is_shutdown(): 
        my_pub = rospy.Publisher(tb3_name+'batterystate_err_inj', BatteryState, queue_size = 10) 
        my_sub = rospy.Subscriber(tb3_name+'battery_state', BatteryState, listener)

        #########################################
        #INJECT ERRORS HERE
        batterystate_msg.header.seq = actual_seq
        batterystate_msg.header.stamp.secs = actual_secs
        batterystate_msg.header.stamp.nsecs = actual_nsecs
        batterystate_msg.header.frame_id = actual_frameid

        batterystate_msg.voltage = actual_voltage
        batterystate_msg.current = actual_current
        batterystate_msg.charge = actual_charge
        batterystate_msg.capacity = actual_capacity
        batterystate_msg.design_capacity = actual_designcapacity
        batterystate_msg.percentage = actual_percentage

        batterystate_msg.power_supply_status = actual_powersupplystatus
        batterystate_msg.power_supply_health = actual_powersupplyhealth
        batterystate_msg.power_supply_technology = actual_powersupplytechnology

        batterystate_msg.present = actual_present
        batterystate_msg.cell_voltage = actual_cellvoltage
        batterystate_msg.location = actual_location
        batterystate_msg.serial_number = actual_serialnumber

        
        #########################################
            
        my_pub.publish(batterystate_msg)
        rate.sleep()
        
    rospy.spin()

if __name__ == '__main__':
    tb3_0 = bs_err_inj("turtlebot1")
    tb3_1 = bs_err_inj("turtlebot2")
    tb3_2 = bs_err_inj("turtlebot3")
    tb3_3 = bs_err_inj("turtlebot4")

