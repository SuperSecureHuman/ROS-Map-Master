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

Now you can send goal to each robot using the 2D Nav Goal button in RViz. The seperately opened Rviz window will now have the map of the environment.

Video (At 3x)

![Multi Robot @3x](.media/MultiBot 3x.mkv)