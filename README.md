# ROS_Self_Driving_Car_Sim
Minimalistic Self Driving Car Simulation with basic Sensors and Perception Tasks


## Installing ROS

  Accept software from packages.ros.org. 

 > sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

Set up keys

 > sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
 
Debian  up-to-date: 
 > sudo apt-get update

Install full ROS version: 
 > sudo apt-get install ros-kinetic-desktop-full


You will need to initialize rosdep:
 > sudo rosdep init

 > rosdep update
 
It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched: 
 > echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
 > source ~/.bashrc

## Installing Gazebo 8

Setup the packages list: 

 > sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable \`lsb_release -cs\` main" > /etc/apt/sources.list.d/gazebo-stable.list'
 
Set up your keys
 > wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

Make sure your Debian package index is up-to-date: 

 > sudo apt-get update
 
Install gazebo 8 and dependencies following next order: 
 > sudo apt-get install libignition-math3 

 > sudo apt-get install gazebo8

 > sudo apt-get install ros-kinetic-gazebo8-ros-pkgs


## Additional requirements

Gazebo packages: 

 > sudo apt-get  install libignition-transport4 libignition-transport4-dev libignition-msgs0-dev

ROS packages:

 > sudo apt-get install  ros-kinetic-laser-geometry ros-kinetic-joint-state-publisher
 
## Create a new Ros Workspace and Compile
> mkdir -p catkin_ws/src

> cd catkin_ws/src

> git clone https://github.com/jediofgever/ROS_Self_Driving_Car_Sim.git

> cd ..

> catkin_make

## Execute Nodes
Two steps here, first run simulator, NOTE; make sure you source this file de-
vel/setup.bash, in your workspace

>source devel/setup.bash 

>roslaunch simulator city.launch

Now  You  should  see  Gazebo  opening  with  Car  at  the  center,  This  might
warm your computer.
in a seperate terminal launch perception and RVIZ, again source devel/setup.bash

>source devel/setup.bash 

>roslaunch perception perception.launch

Now You should see RVIZ open with Car Model, Detected Obstacles , LI-
DAR pointclouds ,Local Costmap and Simulated Camera.
