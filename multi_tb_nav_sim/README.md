# HOW TO USE
## Map Merging  
you will need 3 separate windows to launch:  
```
roslaunch multiple_turtlebots_sim multi_bot.launch
roslaunch multi_nav_bots_sim mult_map_merge_bots.launch
rosrun rviz rviz -d `rospack find turtlebot3_gaze`/rviz/multi_turtlebot3_slam.rviz
```
These set up the 4 robots and their gmapping and map merging nodes. The rviz shows the window with the merged map in it.  

To individually teleoperate any one of the robots use the following:  
```
ROS_NAMESPACE=tb3_0 rosrun turtlebot3_teleop turtlebot3_teleop_key
ROS_NAMESPACE=<tb3 robot id> rosrun turtlebot3_teleop turtlebot3_teleop_key
```

// Note: the rviz command pulls up a window that only shows fields for 3 of the 4 robots  

When you have a map you want to save, run the following command:  
```
rosrun map_server map_saver -f ~/map
```

# TODO NOTES

Navigation stack warning
[ WARN] [1604777659.565045137, 349.565000000]: Timed out waiting for transform from base_footprint to map to become available before running costmap, tf error: Could not find a connection between 'map' and 'base_footprint' because they are not part of the same tree.Tf has two or more unconnected trees.. canTransform returned after 0.1 timeout was 0.1.

running ROS + Gazebo from USB / maybe look at AWS RoboMaker

1. Add all gmapping / slam launch files together
    - gmapping all together but something goes wonky with the combining map
    - separate into separate threads or something
    - edit rviz call to show all robots
2. Check multirobot_map_merge for handling unknown number of robots / map contributors
3. create / save map


4. run navigation of namespaced robot on saved map


Frontier Exploration ROS Package
dropdown for selecting robot
publish a point


rosrun map_server map_saver -f ~/map