#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

turtlebot_dict = {
	"turtlebot1" : "tb3_0/odom",
	"turtlebot2" : "tb3_1/odom",
	"turtlebot3" : "tb3_2/odom",
	"turtlebot4" : "tb3_3/odom"
}

class OdomSubscriber(object):
	def __init__(self, tb3_name):
		topic = turtlebot_dict[tb3_name]
		self.sub = rospy.Subscriber(topic, Odometry, self.sub_callback)
		self.odomdata = Odometry()

	def sub_callback(self, msg):
		self.odomdata = msg
		print(self.odomdata)

if __name__ == "__main__":
	rospy.init_node("odom_subscriber_node", anonymous=True)
	odom_subscriber = OdomSubscriber("turtlebot1")
	rospy.spin()

