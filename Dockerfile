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
RUN apt install ros-humble-turtlebot3  -y 
RUN apt install ros-humble-slam-toolbox -y 
RUN apt install ros-humble-rmw-cyclonedds-cpp -y

RUN add-apt-repository -y  ppa:borglab/gtsam-release-4.1
RUN apt install libgtsam-dev libgtsam-unstable-dev -y
RUN apt install ros-humble-xacro -y
RUN apt install ros-humble-ros2-controllers ros-humble-ros2-control -y

WORKDIR /root/ros_docker

# Clone gazevbo models to speedup gazebo startup time
#RUN git clone --depth 1  --single-branch  https://github.com/osrf/gazebo_models ~/.gazebo/models

# Clone the git repo

RUN git clone --depth 1  https://github.com/SuperSecureHuman/ROS-Map-Master.git '.'

# clone submodules

RUN git submodule update --init --recursive

# Build the workspace

RUN . /opt/ros/humble/setup.sh \
    && colcon build --symlink-install

# Append the content to the existing .bashrc file
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc 

# source /root/ros_docker/install/setup.bash in bashrc
RUN echo "source /root/ros_docker/install/setup.bash" >> /root/.bashrc