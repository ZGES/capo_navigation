# Structure of configuration files and their description

```
|--bt
|  |--test_bt.xml -> example behavior tree
|
|--navigation_params
|  |--nav2_params0.yaml -> NavFn Dijkstra version with DWB
|  |--nav2_params1.yaml -> NavFn A* version with DWB
|  |--nav2_params2.yaml -> Theta* with DBW (dont work on foxy)
|  |--nav2_params3.yaml -> Smac2D Moore's version with DWB
|  |--nav2_params4.yaml -> Smac2D Von Neumann's version with DWB
|  |--nav2_params5.yaml -> SmacHybridA* Dubin's version with DBW
|  |--nav2_params6.yaml -> SmacHybridA* Red-Shepp's version with DBW
|  |--nav2_params7.yaml -> NavFn Dijkstra version with TEB (dont work on foxy)
|  |--nav2_params8.yaml -> NavFn A* version with TEB (dont work on foxy)
|  |--nav2_params9.yaml -> Theta* with TEB (dont work on foxy)
|  |--nav2_params10.yaml -> Smac2D Moore's version with TEB (dont work on foxy)
|  |--nav2_params11.yaml -> Smac2D Von Neumann's version with TEB (dont work on foxy)
|  |--nav2_params12.yaml -> SmacHybridA* Dubin's version with TEB (dont work on foxy)
|  |--nav2_params13.yaml -> SmacHybridA* Red-Shepp's version with TEB (dont work on foxy)
|  |--nav2_params14.yaml -> NavFn Dijkstra version with RPP
|  |--nav2_params15.yaml -> NavFn A* version with RPP
|  |--nav2_params16.yaml -> Theta* with RPP (dont work on foxy)
|  |--nav2_params17.yaml -> Smac2D Moore's version with RPP
|  |--nav2_params18.yaml -> Smac2D Von Neumann's version with RPP
|  |--nav2_params19.yaml -> SmacHybridA* Dubin's version with RPP
|  |--nav2_params20.yaml -> SmacHybridA* Red-Shepp's version with RPP
|
|--source_files
|  |--behavior_tree
|     |--behavior_tree_engine.cpp -> source file of nav2_behavior_tree which fixes bug that stops behavior tree from working after navigation is aborted
|     |--bt_action_node.hpp -> header file of nav2_behavior_tree which fixes bug that stops behavior tree from working after navigation is aborted
|
|  |--planner_server
|     |--planner_server.cpp -> modified nav2_planner source file to allow sending metrics
|     |--planner_server.hpp -> modified nav2_planner header file to allow sending metrics
|
|--ekf.yaml -> configuration for robot_localization's EKF node
|
|--slam_toolbox.yaml -> slam_toolbox configuration
|
|--urg_node.yaml -> laser scanner configuration
```
