# Base image for the container
FROM osrf/ros:humble-desktop

# Setup vscode
RUN apt-get update && apt-get install -y curl

RUN sudo apt-get install wget gpg lsb-release gnupg apt-transport-https python3-pip software-properties-common -y
RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt update -y
RUN apt upgrade -y
RUN apt install ros-humble-gazebo-ros-pkgs -y
RUN apt install ros-humble-navigation2 ros-humble-nav2-bringup -y 
RUN apt install ros-humble-turtlebot3-gazebo  ros-humble-turtlebot3  -y 
RUN apt install ros-humble-slam-toolbox -y 
RUN apt install ros-humble-rmw-cyclonedds-cpp -y

RUN add-apt-repository -y  ppa:borglab/gtsam-release-4.1
RUN apt install libgtsam-dev libgtsam-unstable-dev -y
RUN apt install ros-humble-twist-mux -y
RUN apt install ros-humble-ros2-controllers ros-humble-ros2-control -y

WORKDIR /root/ros_docker

# Clone gazevbo models to speedup gazebo startup time
RUN git clone --depth 1  --single-branch  https://github.com/osrf/gazebo_models ~/.gazebo/models

# Append the content to the existing .bashrc file
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "export TURTLEBOT3_MODEL=waffle" >> /root/.bashrc \
    && echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> /root/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc 
