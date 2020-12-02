#!/usr/bin/env python

import rospy
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid

class NavSubscriber(object):
	def __init__(self):
		self.sub = rospy.Subscriber('map', OccupancyGrid, self.sub_callback)
		self.mapdata = OccupancyGrid()

	def sub_callback(self, msg):
		self.mapdata = msg
		print(self.mapdata)

if __name__ == "__main__":
	rospy.init_node("nav_subscriber_node", anonymous=True)
	nav_subscriber = NavSubscriber()
	rospy.spin()

