#!/usr/bin/env python
# https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

import rospy
import math
import json

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from id_msg.msg import CustomId

OFFSET_VAL = 10
STD_W_ORIENTATION = 1.0
t0 = "tb3_0"
NUMBER_OF_TURTLEBOTS = 4
DISTANCE_SET = 0.7

WORKING = str(False)
FAILING = str(True)

X_TARG = 6
Y_TARG = -1

working_turtlebot_names = {
    "tb3_0" : WORKING,
	"tb3_1" : WORKING,
	"tb3_2" : WORKING,
	"tb3_3" : WORKING
}

turtlebot_names = {
    "turtlebot1" : "tb3_0",
	"turtlebot2" : "tb3_1",
	"turtlebot3" : "tb3_2",
	"turtlebot4" : "tb3_3"
}

turtlebot_return_positions = {
    "tb3_0" : (-1,4),
	"tb3_1" : (-1,1),
	"tb3_2" : (-4,4),
	"tb3_3" : (-4,1)
}

def read_live_tbots():
    with open('/home/widogast/tbot_data.json') as json_file:
        data = json.load(json_file)
        print(data)
        for tbot in data.keys():
            working_turtlebot_names[tbot] = data[tbot]

def to_rad(ang):
    return ang/180*3.1459

def create_x_y(ang, start_x, start_y):
    xx = start_x + DISTANCE_SET * math.cos(to_rad(ang))
    yy = start_y + DISTANCE_SET * math.sin(to_rad(ang))
    return (xx,yy)

def multiple_move_goals(x_m, y_m):
    read_live_tbots()
    acc_clients = {}
    for count,tbot in enumerate(turtlebot_names.keys()):
        bot_angle = count * (360 / NUMBER_OF_TURTLEBOTS)
        (x_new, y_new) = create_x_y(bot_angle, x_m, y_m)

        if working_turtlebot_names[turtlebot_names[tbot]] != WORKING:
            x_new = turtlebot_return_positions[turtlebot_names[tbot]][0]
            y_new = turtlebot_return_positions[turtlebot_names[tbot]][1]

        acc_clients[tbot] = movebase_client(turtlebot_names[tbot], x_new, y_new)
        if tbot in acc_clients:
            # rospy.loginfo("Goal execution done!")
            rospy.loginfo("goal pushed to " + tbot + "; target: " + str(x_new) + ", " + str(y_new))

def movebase_client(robot_name, target_x, target_y):
    move_goal_topic = robot_name + "/move_base"

   # Create an action client called "move_base" with action definition file "MoveBaseFAction"
    client = actionlib.SimpleActionClient(move_goal_topic,MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = OFFSET_VAL + target_x
    goal.target_pose.pose.position.y = OFFSET_VAL + target_y
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = STD_W_ORIENTATION

   # Sends the goal to the action server.
    client.send_goal(goal)
    return client
#    # Waits for the server to finish performing the action.
#     wait = client.wait_for_result()
#    # If the result doesn't arrive, assume the Server is not available
#     if not wait:
#         rospy.logerr("Action server not available!")
#         rospy.signal_shutdown("Action server not available!")
#     else:
#     # Result of executing the action
#         return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        multiple_move_goals(X_TARG, Y_TARG)
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")



        # rostopic pub tb3_0/move_base/cancel actionlib_msgs/GoalID -- {}