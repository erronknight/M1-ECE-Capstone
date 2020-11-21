#! /usr/bin/env python

import rospy
from id_msg.msg import CustomId

def publish_custom_msg():
	rospy.init_node('custom_msg_publisher', anonymous=True)
	pub = rospy.Publisher('/id_msg', CustomId, queue_size=1)
	rate = rospy.Rate(2)
	message = CustomId()
	message.id = "1"
	message.name = "test_name"
	message.fail_flag = "working"

	pub.publish(message)
	rate.sleep()

if __name__ == '__main__':
	
	while not rospy.is_shutdown():
		pub_msg = publish_custom_msg()


