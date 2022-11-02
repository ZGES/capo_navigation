# Capo_navigation
Repository consist of two packages - `capo_description` with URDF description of the CAPO robot and all neccessary configuration files and `metrics_collector` for gathering information about computed path length and path computing time from global planners algorithms.

## capo_description
The `src/description` directory contains URDF of the CAPO robot.

In `config` directory configuration for used navigation algorithms are presented as well as configuration files for other libraries. For more details go [here](https://github.com/ZGES/capo_navigation/tree/master/capo_description/config).

The `launch` directory contains launch file that starts robot_state_publisher and joint_state_publisher packages for robot description sharing and robot_localization for odometry information smoothing and `odom -> base_link` transformation publishing.

## metrics_collector
Gathers information about path computation time and path length send by global planners.

## Installation and configuration
To install and configure ROS2 on your robot look [here](https://docs.ros.org/en/foxy/index.html).

When ROS configuration is done create working directory and head to `src` subdirectory 
```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```
Now clone neccessary libraries:
1. capo_navigation
```
git clone https://github.com/ZGES/capo_navigation.git
```
2. [ros2_roboclaw](https://github.com/ZGES/ros2_roboclaw) - ROS drivers for Roboclaw
```
git clone https://github.com/ZGES/ros2_roboclaw.git
```
3. [navigation2](https://github.com/ros-planning/navigation2/tree/foxy-devel) - ROS navigation stack
```
git clone https://github.com/ros-planning/navigation2.git -b foxy-devel
```
4. [urg_node](https://github.com/ros-drivers/urg_node/tree/foxy-devel) - ROS drivers for Hokuyo laser scanner
```
git clone https://github.com/ros-drivers/urg_node.git -b foxy-devel
```

Install binary versions of:
1. [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox/tree/foxy-devel) - library for SLAM used during mapping
```
sudo apt install ros-foxy-slam-toolbox
```
2. [robot_localization](https://docs.ros.org/en/noetic/api/robot_localization/html/index.html) - implements EKF node
```
apt install ros-foxy-robot_localization
```

Before attempting to build libraries you need to replace some files:
1. Replace the content of the `navigation2/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml` with `capo_navigation/capo_description/config/bt/test_bt.xml`
2. Replace the content of `urg_node/launch/urg_node_serial.yaml` with `capo_navigation/capo_description/config/urg_node.yaml`
3. Replace `navigation2/nav2_behavior_tree/src/behavior_tree_engine.cpp` with `capo_navigation/capo_description/config/source_files/behavior_tree/behavior_tree_engine.cpp` and `navigation2/nav2_behavior_tree/include/nav2_behavior_tree/bt_action_node.hpp` with `capo_navigation/capo_description/config/source_files/behavior_tree/bt_action_node.hpp`
4. Replace `navigation2/nav2_planner/src/planner_server.cpp` with `capo_navigation/capo_description/config/source_files/planner_server/planner_server.cpp` and `navigation2/nav2_planner/include/nav2_planner/planner_server.hpp` with `capo_navigation/capo_description/config/source_files/planner_server/planner_server.hpp`

Now you can check needed dependencies
```
cd ~/capo_navigation
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy
```

When all dependencies are installed you can attempt to build the workspace
```
colcon build --symlink-install
```

### Rviz
It is adviced to use Rviz to ease sending the navigation commands and monitoring the robot behaviour. To do so install ROS on your machine (reccomended methods for [ubuntu](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)/[windows](https://ms-iot.github.io/ROSOnWindows/GettingStarted/SetupRos2.html)).
When ROS is installed create working directory, clone [capo_rviz_launch](https://github.com/ZGES/capo_rviz_launch) to `src` subdirectory and build it.
```
git clone https://github.com/ZGES/capo_rviz_launch.git
```

## How to use
From your computer create 5 ssh sessions connected with the robot. In each terminal window move to working directory and source local versions of libraries
```
cd dev_ws
. install/local_setup.bash
```
Start these 3 packages in any order each in separate terminal window:
1. Description of the robot
```
ros2 launch capo_description describe.launch.py
```
2. Laser scanner drivers
```
ros2 launch urg_node urg_node_launch.py
```
3. Roboclaw drivers
```
ros2 run ros2_roboclaw steer
```

When all of the above are running
* for mapping launch:
1. Slam_toolbox
```
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false param_file:=../slam_toolbox.yaml
```
2. Nav2 stack (change number following nav2_params to change used algorithms)
```
ros2 launch nav2_bringup navigation_launch.py params_file:=src/capo_navigation/capo_description/config/navigation_params/nav2_params0.yaml use_sim_time:=false
```

* for navigation:
1. Metrics collection (optional)
```
ros2 run metrics_collector metric
```
2. Nav2 stack (change number following nav2_params to change used algorithms, provide valid path to the map)
```
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=False map:=/home/ubuntu/map.yaml params_file:=src/capo_navigation/capo_description/config/navigation_params/nav2_params0.yaml use_sim_time:=false
```

### Useful commands
1. Save the map
```
ros2 run nav2_map_server map_saver_cli -f ~/map
```
2. Save transformations graph
```
ros2 run tf2_tools view_frames.py
```
3. Manually send command to set wheels velocities (there can not be messages from navigation stack to roboclaws for this to take effect)
```
ros2 topic pub --once /manual roboclaw_comm/msg/Speed "{speed: [0,0,0,0]}"
```
