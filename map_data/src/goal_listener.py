#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

class GoalSubscriber(object):
	def __init__(self):
		self.sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.sub_callback)
		self.goaldata = PoseStamped()

	def sub_callback(self, msg):
		self.goaldata = msg
		print(self.goaldata)

if __name__ == "__main__":
	rospy.init_node("goal_subscriber_node", anonymous=True)
	goal_subscriber = GoalSubscriber()
	rospy.spin()

