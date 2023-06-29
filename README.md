# ROS Map Master

## Overview

This ROS project explores the extents of mapping and localization in ROS2.

## Invloved  Components

### Multi-Robot Mapping (2d)

Leverages SLAM and Nav2 stack to map an environment with multiple robots. The robots are able to localize themselves in the map and navigate to a goal location. By having multiple goal locations for each robots, we can have the robots explore the environment and map it. The robots are able to avoid obstacles while navigating to the goal locations.

#### Launching the Multi-Robot Mapping

Pre-requisites (for the current phase of the project):

* Nav2 Stack
* SLAM Toolbox
* Slam Gmapping
* Turtlebot3 Stack (Check issues for issues with our custom robot)

To launch the multi-robot mapping, run the following command:

```bash
ros2 launch multirobot_map_merge multi_tb3_simulation_launch.py
ros2 launch multirobot_map_merge map_merge.launch.py
```

Launch RViz with the following command:

```bash
rviz2 -d map_merge/launch/map_merge.rviz
```

You can send a goal to each robot using the 2D Nav Goal button in RViz. The separately opened Rviz window will now have the map of the environment.




#### Issues

While you can use custom robots for this, we had an issue where our robot would randomly keep moving in its own way, even after reaching the nav goal. This forced us to stick with Turtlebot for the demo.


https://github.com/SuperSecureHuman/ROS-Map-Master/assets/88489071/e9a427b3-4237-465b-b086-f7fa7560aa27


### Autonomous Exploration

Uses [Frontier Exploration](https://arxiv.org/pdf/1806.03581.pdf) from this repo - [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2), with some modifications to the code to avoid some crashes we were facing.

Ideally, this can be combined with multi-bot mapping to have multiple robots explore and map the environment, but this needs more resources than we had available.

#### Launching the Autonomous Exploration

Pre-requisites (for the current phase of the project):
* Nav2 Stack
* SLAM Toolbox

To launch the autonomous exploration, run the following command:

Launch any world in gazebo, with any Nav2 working robot. Then run the following commands:

```bash
ros2 launch nav2_bringup navigation_launch.py
ros2 launch slam_toolbox online_async_launch.py
ros2 launch nav2_bringup rviz_launch.py
ros2 launch explore_lite explore.launch.py
```


https://github.com/SuperSecureHuman/ROS-Map-Master/assets/88489071/55cd3c56-19b5-486b-81ee-557c3a60b844


### 3d Reconstruction

Uses [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) to reconstruct the environment in 3d. PointCloud2d data is from Velodyne 16P, with a custom fork found here - [Velodyne_Simulator](https://github.com/SuperSecureHuman/velodyne_simulator) (Branch Humble-Dev). The robot is custom-made. 

https://github.com/SuperSecureHuman/ROS_Workspace_Private/assets/88489071/5fccd372-4c06-47f3-857d-f11b15bfccc6

Pre-requisites (for the current phase of the project):

* GTSAM (Compile from Source for better stability)
* Velodyne Simulator (Use the branch mentioned above)
* LIO-SAM
* Teleop for controls

To launch the 3d reconstruction, run the lidar_bot we provided with a world loaded. Then do the following:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 launch lio_sam run.launch.py
```

#### Issues

There are some stray random outputs from the gazebo IMU, which we heavily suspect causes occasionally unstable mapping.


https://github.com/SuperSecureHuman/ROS-Map-Master/assets/88489071/79bdd961-e51f-4ae4-8f8a-df98c6eebc7e


### The Car Model

We built a car, from the ground up, without using any pre-built models. Just with simple shapes, we could get the car outline done.

Then, we added the following sensors:

* Camera
* 2d Lidar (from gazebo)
* IMU (from gazebo)
* 3d Lidar (from Velodyne Simulator)
* Differential drive with ROS Control

The car can move around in the environment, and the sensors can provide data to the ROS system.

## Issues Faced (Major ones)

* [SOLVED] Velodyne kept crashing the system, and we had to use a custom fork of the simulator and modify the tf tree and Gaussian kernel to be more memory efficient. This largely solved the issue in my case, while other members had to reduce the number of rays and update the sensor frequency.

* [SOLVED] Crashes when using explore node. This was solved by modifying the BFS algorithm to be more memory efficient.

* The robot moves randomly, sometimes failing to reach the nav goal. This is a major issue; we are still trying to determine the cause. If this issue is sorted, then all the uses of the turtle bot can be replaced with this bot, and we can have a fully custom robot.

## Possible Future Work

[ ] Multi Robot Exploration - Currently entire stack crashes when trying to launch exploration with multiple bot. Need to first confirm if its a resource availablity or code optimization issue.

[ ] Map Merge for 3d
