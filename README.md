# capo_navigation
Repository with Nav2 library configured for CAPO robot and additional packages used to navigate.

1. [navigation2](https://github.com/ros-planning/navigation2/tree/foxy-devel) - Nav2 package, in this repo with modified configuration to run with CAPO robot. More about Nav2 [here](https://navigation.ros.org/).
2. [urg_node](https://github.com/ros-drivers/urg_node/tree/foxy-devel) - ROS2 package with Hokuyo laser scaner drivers.
3. [cartographer](https://github.com/ros2/cartographer/tree/foxy) - ROS2 SLAM package.
4. [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox/tree/foxy-devel) - ROS2 SLAM package.
5. [ros2_roboclaw](https://github.com/ZGES/ros2_roboclaw) - ROS2 wrapper for [amber](https://github.com/project-capo/amber-python-drivers) roboclaw drivers.

## Installation and configuration
For information how to install, configure and work with ROS2 look [here](https://docs.ros.org/en/foxy/index.html).

With ROS installed clone this repository and move to capo_navigation folder which will be working directory

    
    git clone https://github.com/ZGES/capo_navigation.git
    cd ~/capo_navigation
    

Due to problems with build slam_toolbox package you need to install binaries
    
    sudo apt install ros-foxy-slam-toolbox
   
or clone [this repository](https://github.com/SteveMacenski/slam_toolbox/tree/foxy-devel) into src directory and try to build it yourself
    
    cd ~/capo_navigation/src
    git clone https://github.com/SteveMacenski/slam_toolbox.git -b foxy-devel
    

Run rosdep install to install all needed dependencies
    
    cd ~/capo_navigation
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy
    
When all dependencies are installed build the project

    colcon build --symlink-install

## How to use
TODO
