#! /bin/sh

catkin build -j4
source devel/setup.bash
roslaunch turtlebot3_control demo.launch


