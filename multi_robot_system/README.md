NEW ERROR
[ WARN] [1607407853.141989466, 1486.243000000]: The origin for the sensor at (-1.01, 4.01) is out of map bounds. So, the costmap cannot raytrace for it.


possibly could be solved with an intermediate tf transform

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


<launch>

    <!-- ROBOT 1 -->
    <include file="$(find multi_robot_system)/launch/map_single_robot.launch">
        <arg name="robot_name" value="tb3_0" />
        <arg name="robot_x_pos" value=" -1.0"/>
        <arg name="robot_y_pos" value=" 4.0"/>
        <arg name="robot_z_pos" value=" 0.0"/>
        <arg name="robot_yaw" value=" 0.0"/>
    </include>

    <!-- ROBOT 2 -->
    <include file="$(find multi_robot_system)/launch/map_single_robot.launch">
        <arg name="robot_name" value="tb3_1" />
        <arg name="robot_x_pos" value=" -1.0"/>
        <arg name="robot_y_pos" value=" 1.0"/>
        <arg name="robot_z_pos" value=" 0.0"/>
        <arg name="robot_yaw" value=" 0.0"/>
    </include>

    <!-- ROBOT 3 -->
    <include file="$(find multi_robot_system)/launch/map_single_robot.launch">
        <arg name="robot_name" value="tb3_2" />
        <arg name="robot_x_pos" value=" -4.0"/>
        <arg name="robot_y_pos" value=" 4.0"/>
        <arg name="robot_z_pos" value=" 0.0"/>
        <arg name="robot_yaw" value=" 0.0"/>
    </include>

    <!-- ROBOT 3 -->
    <include file="$(find multi_robot_system)/launch/map_single_robot.launch">
        <arg name="robot_name" value="tb3_3" />
        <arg name="robot_x_pos" value=" -4.0"/>
        <arg name="robot_y_pos" value=" 1.0"/>
        <arg name="robot_z_pos" value=" 0.0"/>
        <arg name="robot_yaw" value=" 0.0"/>
    </include>

    <node pkg="multirobot_map_merge" type="map_merge" respawn="false" name="map_merge" output="screen">
        <param name="robot_map_topic" value="map"/>
        <param name="robot_namespace" value="tb3"/>
        <param name="merged_map_topic" value="map"/>
        <param name="world_frame" value="map"/>
        <param name="known_init_poses" value="true"/>
        <param name="merging_rate" value="0.5"/>
        <param name="discovery_rate" value="0.05"/>
        <param name="estimation_rate" value="0.1"/>
        <param name="estimation_confidence" value="1.0"/>
    </node>


    <!-- <node pkg="tf" type="static_transform_publisher" name="world_to_tb3_0_tf_broadcaster" args="0 0 0 0 0 0 /map /tb3_0/map 100"/>
    <node pkg="tf" type="static_transform_publisher" name="world_to_tb3_1_tf_broadcaster" args="0 0 0 0 0 0 /map /tb3_1/map 100"/>
    <node pkg="tf" type="static_transform_publisher" name="world_to_tb3_2_tf_broadcaster" args="0 0 0 0 0 0 /map /tb3_2/map 100"/>
    <node pkg="tf" type="static_transform_publisher" name="world_to_tb3_3_tf_broadcaster" args="0 0 0 0 0 0 /map /tb3_3/map 100"/> -->

</launch>