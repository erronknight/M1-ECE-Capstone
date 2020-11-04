#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

class OdomSubscriber(object):
	def __init__(self):
		self.sub = rospy.Subscriber('/odom', Odometry, self.sub_callback)
		self.odomdata = Odometry()

	def sub_callback(self, msg):
		self.odomdata = msg
		print(self.odomdata)

if __name__ == "__main__":
	rospy.init_node("odom_subscriber_node", anonymous=True)
	odom_subscriber = OdomSubscriber()
	rospy.spin()

