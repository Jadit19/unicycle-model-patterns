# SURGE

## Software requirements
The program was tried and tested on a system with the following specifications:

1. Ubuntu 20.04
2. ROS1-Noetic
3. Gazebo 11
4. GNU gcc-17
5. Python 3.8.10

## Installation and Setup
In order to install this, here is the procedure to be followed:

1. Clone the repository

```sh
git clone https://github.com/Jadit19/SURGE.git
cd SURGE
catkin init
cd src
```

2. Setting up
```sh
chmod +x ./scripts/dependencies.sh
chmod +x ./scripts/install.sh
./scripts/dependencies.sh
./scripts/install.sh
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
```

3. Building and launching

```sh
catkin build -j4
source ../devel/setup.bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```