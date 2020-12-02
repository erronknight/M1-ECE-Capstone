#! /usr/bin/env python

import rospy
from id_msg.msg import CustomId

def callback(data):
	rospy.loginfo("I heard %s", data.fail_flag)

def subscribe_custom_msg():
	rospy.init_node('custom_msg_subscriber', anonymous=True)
	pub = rospy.Subscriber('/id_msg', CustomId, callback)

	rospy.spin()

if __name__ == '__main__':
	subscribe_custom_msg()
