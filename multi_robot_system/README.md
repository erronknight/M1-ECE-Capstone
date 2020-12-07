[ WARN] [1607327055.200716635, 148.438000000]: Timed out waiting for transform from base_footprint to map to become available before running costmap, tf error: canTransform: source_frame base_footprint does not exist.. canTransform returned after 0.104 timeout was 0.1.


Run

Either of the following
```
roslaunch multiple_turtlebots_sim multi_bot.launch
roslaunch multi_robot_system main.launch
```

then
```
roslaunch multi_robot_system merge_map_robots.launch
roslaunch multi_robot_system navigation.launch

```