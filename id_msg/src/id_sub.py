#! /usr/bin/env python

import rospy
from id_msg.msg import CustomId

import json

WORKING = str(False)
FAILING = str(True)

tbot_data = {}

tbot_names = [
	"tb3_0",
	"tb3_1",
	"tb3_2",
	"tb3_3"
]

def callback(data):
	rospy.loginfo("I heard %s", data.fail_flag)
	# print("name " + data.name)
	try:
		if tbot_data[tbot_names[int(data.id)]] != data.fail_flag:
			tbot_data[tbot_names[int(data.id)]] = data.fail_flag
	except:
		rospy.loginfo("could not read")
	with open('/home/widogast/tbot_data.json', 'w') as outfile:
		json.dump(tbot_data, outfile)
		rospy.loginfo("sent to file")


def subscribe_custom_msg():
	rospy.init_node('custom_msg_subscriber', anonymous=True)
	pub = rospy.Subscriber('/id_msg', CustomId, callback)
	rospy.spin()

if __name__ == '__main__':
	tbot_data = {
		"tb3_0" : WORKING,
		"tb3_1" : WORKING,
		"tb3_2" : WORKING,
		"tb3_3" : WORKING
	}
	subscribe_custom_msg()
