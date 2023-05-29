# Base image for the container
FROM osrf/ros:humble-desktop

# Setup vscode
RUN apt-get update && apt-get install -y curl

RUN sudo apt-get install wget gpg lsb-release gnupg apt-transport-https python3-pip software-properties-common -y
RUN wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
RUN sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
RUN sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN rm -f packages.microsoft.gpg
RUN apt update -y
RUN apt install ros-humble-gazebo-ros-pkgs -y
RUN apt install code  -y

# Some Toolboxes
RUN apt install ros-humble-navigation2 -y 
RUN apt install ros-humble-nav2-bringup -y 
RUN apt install ros-humble-turtlebot3-gazebo -y 
RUN apt install ros-humble-turtlebot3 -y 
RUN apt install ros-humble-slam-toolbox -y 

WORKDIR /root/ros_docker

# Append the content to the existing .bashrc file
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "export TURTLEBOT3_MODEL=waffle" >> /root/.bashrc \
    && echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> /root/.bashrc

