# ROS Map Master

## Overview

ROS Map Master is an exploratory ROS project that delves into mapping and localization capabilities within ROS2.

## Involved Components

### Multi-Robot Mapping (2D)

https://github.com/SuperSecureHuman/ROS-Map-Master/assets/88489071/e9a427b3-4237-465b-b086-f7fa7560aa27

This component utilizes SLAM (Simultaneous Localization and Mapping) and the Nav2 stack to enable mapping of an environment with multiple robots. Each robot can localise itself within the map and navigate to designated goal locations. By assigning distinct goal locations to each robot, the system facilitates comprehensive exploration and mapping of the environment. Additionally, the robots are equipped with obstacle-avoidance capabilities during navigation.

#### Launching Multi-Robot Mapping

Prerequisites (for the current phase of the project):

* Nav2 Stack
* SLAM Toolbox
* Slam Gmapping
* Turtlebot3 Stack (Refer to the issues section for details regarding our custom robot)

To launch the multi-robot mapping, execute the following command:

```bash
ros2 launch multirobot_map_merge multi_tb3_simulation_launch.py
ros2 launch multirobot_map_merge map_merge.launch.py
```

Launch RViz using the following command:

```bash
rviz2 -d map_merge/launch/map_merge.rviz
```

You can send a goal to each robot using the "2D Nav Goal" button in RViz. The separate RViz window will display the map of the environment.

#### Issues

While it is possible to use custom robots, we encountered an issue where our robot would intermittently continue moving in its own direction even after reaching the navigation goal. So, we decided to use the Turtlebot for the demo.

### Autonomous Exploration


https://github.com/SuperSecureHuman/ROS-Map-Master/assets/88489071/55cd3c56-19b5-486b-81ee-557c3a60b844

This component utilizes the [Frontier Exploration](https://arxiv.org/pdf/1806.03581.pdf) package from the repository [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2), with certain modifications to address specific crashes we encountered.

Ideally, this component can be combined with multi-bot mapping to enable multiple robots to explore and map the environment simultaneously. However, this requires more resources than were available to us.

#### Launching Autonomous Exploration

Prerequisites (for the current phase of the project):
* Nav2 Stack
* SLAM Toolbox

To launch autonomous exploration, execute the following commands:

Launch any world in Gazebo with a functional Nav2-supported robot. Then run the following commands:

```bash
ros2 launch nav2_bringup navigation_launch.py
ros2 launch slam_toolbox online_async_launch.py
ros2 launch nav2_bringup rviz_launch.py
ros2 launch explore_lite explore.launch.py
```

### 3D Reconstruction

https://github.com/SuperSecureHuman/ROS-Map-Master/assets/88489071/79bdd961-e51f-4ae4-8f8a-df98c6eebc7e

This component employs [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) to reconstruct the environment in 3D. PointCloud2D data is obtained from Velodyne 16P, utilizing a custom fork available at [Velodyne_Simulator](https://github.com/SuperSecureHuman/velodyne_simulator) (Branch: Humble-Dev). The robot used in this component is custom-made.

Prerequisites (for the current phase of the project):

* GTSAM (Compile from Source for better stability)
* Velodyne Simulator (Use the aforementioned branch)
* LIO-SAM
* Teleop for controls

To launch 3D reconstruction, run the provided lidar_bot in Gazebo with a loaded world. Then follow these steps:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 launch lio_sam run.launch.py
```

#### Issues

Occasionally, the Gazebo IMU generates random stray outputs that can cause unstable mapping. This issue is currently under investigation.


### The Car Model

We have built a custom car model from scratch, abstaining from using pre-built models. By utilizing simple shapes, we were able to construct the car outline. The car model incorporates the following sensors:

* Camera
* 2D Lidar (from Gazebo)
* IMU (from Gazebo)
* 3D Lidar (from Velodyne Simulator)
* Differential drive with ROS Control

The car can navigate within the environment while its sensors provide data to the ROS system.

## Major Issues Encountered

* [SOLVED] Velodyne crashes the system: We resolved this issue by using a custom fork of the simulator, modifying the tf tree and Gaussian kernel to enhance memory efficiency. While some team members also had to reduce the number of rays and adjust the sensor frequency, these modifications largely resolved the problem for us.

* [SOLVED] Crashes during explore node usage: We addressed this issue by optimizing the BFS algorithm to improve memory efficiency.

* Random robot movements and occasional failure to reach the navigation goal: This major issue is still being investigated. Once resolved, the custom robot can replace the Turtlebot, enabling the use of a fully customized robot.

## Possible Future Work

- [ ] Multi-Robot Exploration: Currently, the entire stack crashes when attempting to launch exploration with multiple robots. To address this, we must ascertain whether it is due to resource availability or requires code optimization.
- [ ]  3D Map Merge: Explore the possibility of implementing map merging capabilities for 3D maps.

- [ ] 3D Map Merge: Explore the possibility of implementing map merging capabilities for 3D maps.

Feel free to reach out for any questions or suggestions!
