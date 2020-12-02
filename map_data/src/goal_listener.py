#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

turtlebot_dict = {
	"turtlebot1" : "tb3_0/move_base_simple/goal",
	"turtlebot2" : "tb3_1/move_base_simple/goal",
	"turtlebot3" : "tb3_2/move_base_simple/goal",
	"turtlebot4" : "tb3_3/move_base_simple/goal"
}

class GoalSubscriber(object):
	def __init__(self, tb3_name):
		topic = turtlebot_dic[tb3_name]
		self.sub = rospy.Subscriber(topic, PoseStamped, self.sub_callback)
		self.goaldata = PoseStamped()

	def sub_callback(self, msg):
		self.goaldata = msg
		print(self.goaldata)

if __name__ == "__main__":
	rospy.init_node("goal_subscriber_node", anonymous=True)
	goal_subscriber = GoalSubscriber("turtlebot1")
	rospy.spin()

