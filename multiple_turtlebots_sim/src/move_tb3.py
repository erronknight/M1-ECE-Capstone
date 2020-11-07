#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Accel
from enum import Enum	

turtlebot_dict = {
	"turtlebot1" : "tb3_0/cmd_vel",
	"turtlebot2" : "tb3_1/cmd_vel",
	"turtlebot3" : "tb3_2/cmd_vel"	
}

# Input: str, str, double
# Takes in the name of the turtlebot, the instruction for movement
# and the movement amount. The function then sets up the message
# and publishes to the topic that corresponds with the given robot.
def move_tb3(tb3_name, instr, move_amt):
	# set up topic and rate for publishing the message
	topic = turtlebot_dict[tb3_name]
    	pub = rospy.Publisher(topic, Twist, queue_size=1)
    	rate = rospy.Rate(1) # 1hz
	move = Twist()
	
	# set up the message
	if instr == "rotate":
		move.angular.z = move_amt
	elif instr == "drive":
		move.linear.x = move_amt
	else:
		print("Error: Incorrect instruction type")
    	
	pub.publish(move)

if __name__ == '__main__':
	rospy.init_node("move_tb3", anonymous=True)

	while not rospy.is_shutdown():
		tb3_0 = move_tb3("turtlebot1", "rotate", 0)
		tb3_1 = move_tb3("turtlebot2", "rotate", 0)
		tb3_2 = move_tb3("turtlebot3", "drive", 0)
