#!/usr/bin/env python
# liscense removed for brevity

import rospy
from geometry_msgs.msg import Twist

def move_tb3(tb3_num, rot_amount_z):
	# set up topic and rate for publishing the message
	topic = 'tb3_' + str(tb3_num) + '/cmd_vel'
    	pub = rospy.Publisher(topic, Twist, queue_size=1)
    	rate = rospy.Rate(1) # 1hz
	
	# set up the message
	rotZ = Twist()
	rotZ.angular.z = rot_amount_z
    	
	pub.publish(rotZ)

if __name__ == '__main__':
	rospy.init_node("rotate_tb3", anonymous=True)

	while not rospy.is_shutdown():
		tb3_0 = move_tb3(0, -0.5)
		tb3_1 = move_tb3(1, .5)
		tb3_2 = move_tb3(2, -5)
