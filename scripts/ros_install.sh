#!/bin/bash

export DEBIAN_FRONTEND=noninteractive

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update

sudo apt -y upgrade

sudo apt install -y ros-melodic-desktop-full ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard \
  ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
  ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-amcl ros-melodic-map-server \
  ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
  ros-melodic-rqt-image-view ros-melodic-navigation ros-melodic-interactive-markers ros-melodic-compressed-image-transport \
  ros-melodic-rosserial-msgs ros-melodic-cartographer

echo "/opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc

sudo mkdir -p /opt/bots/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git /opt/bots/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git /opt/bots/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git /opt/bots/catkin_ws/src

sudo chown -R ubuntu:ubuntu /opt/bots
cd /opt/bots/catkin_ws/
catkin_make
source devel/setup.bash

sudo rosdep init
rosdep update

# roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
# roslaunch turtlebot3_gazebo turtlebot3_world.launch
