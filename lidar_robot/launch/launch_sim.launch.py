from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    package_name = 'lidar_robot'  # <--- CHANGE ME
    urdf_path = "/root/ros_docker/curly-succotash/src/lidar_robot/description/robot.urdf"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )


    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', urdf_path],
                        output='screen')

    # Launch Rviz2 with fixed frame and TF
    rviz2 = Node(package='rviz2', executable='rviz2',
                 arguments=['-d', os.path.join(
                     get_package_share_directory(package_name), 'config', 'my_rviz_config.rviz'),
                            'Fixed Frame:=odom'],
                 output='screen')
    

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,  # Uncomment this line to spawn the robot
        rviz2,
    ])


if __name__ == '__main__':
    generate_launch_description()
