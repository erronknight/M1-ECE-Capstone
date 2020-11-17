TODO and Notes


Navigation stack warning
[ WARN] [1604777659.565045137, 349.565000000]: Timed out waiting for transform from base_footprint to map to become available before running costmap, tf error: Could not find a connection between 'map' and 'base_footprint' because they are not part of the same tree.Tf has two or more unconnected trees.. canTransform returned after 0.1 timeout was 0.1.

running ROS + Gazebo from USB / maybe look at AWS RoboMaker

1. Add all gmapping / slam launch files together
    - gmapping all together but something goes wonky with the combining map
2. Check multirobot_map_merge for handling unknown number of robots / map contributors
3. create / save map


4. run navigation of namespaced robot on saved map


Frontier Exploration ROS Package
dropdown for selecting robot
publish a point


rosrun map_server map_saver -f ~/map