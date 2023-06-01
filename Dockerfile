# Base image for the container
FROM osrf/ros:humble-desktop

# Setup vscode
RUN apt-get update && apt-get install -y curl

RUN sudo apt-get install wget gpg lsb-release gnupg apt-transport-https python3-pip software-properties-common -y
RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt update -y
RUN apt install ros-humble-gazebo-ros-pkgs -y
RUN apt install ros-humble-navigation2 ros-humble-nav2-bringup -y 
RUN apt install ros-humble-turtlebot3-gazebo  ros-humble-turtlebot3  -y 
RUN apt install ros-humble-slam-toolbox -y 
RUN apt install ros-humble-rmw-cyclonedds-cpp -y

WORKDIR /root/ros_docker

# Append the content to the existing .bashrc file
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "export TURTLEBOT3_MODEL=waffle" >> /root/.bashrc \
    && echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> /root/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc 

