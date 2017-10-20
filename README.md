# ENPM808X - ROS Beginner Tutorials
This is the implementation of several ROS beginner tutorials:
1. [Creating a Workspace for Catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
2. [Creating a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
3. [Building a ROS Package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)
4. [Creating a ROS msg and srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
5. [Writing a Simple Publisher and Subscriber (C++)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

The goal of the project was to install ROS and Catkin, create a workspace, and create a simple publisher and subscriber package.

## Getting Started

### Installing Dependencies
The beginner tutorial has several dependencies including ROS Kinetic Kame and Catkin.

#### Installing ROS Kinetic Kame
Full instructions for installing ROS Kinetic Kame can be found [here](http://wiki.ros.org/kinetic/Installation).

The following instructions are for an Ubuntu installation.

1. Set up your sources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

2. Set up your keys
```
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

3. Installation
Make sure your Debian package is up to date:
```
sudo apt-get update
```

Perform a full desktop install:
```
sudo-apt-get install ros-kinetic-desktop-full
```

4. Initialize rosdep
```
sudo rosdep init
rosdep update
```

5. Environment set up
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

6. Install rosinstall
```
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

#### Installing Catkin
Full instructions for installing Catkin can be found [here](www.ros.org/wiki/catkin#Installing_catkin).

```
sudo apt-get install ros-lunar-catkin
```

#### Installing the beginner tutorial from this repository
You will want to navigate to your Catkin workspace's src directory
```
cd ~/catkin_ws/src
```
This is the location you will want to add your new ROS package.
```
git clone --recursive https://github.com/jeshoward/beginner_tutorials
```

Navigate back up to your Catkin workspace directory:
```
cd ~/catkin_ws/
```

Next, you will want to build the project:
```
catkin_make
```

### Running the project
The steps for testing your new ROS package can be found in the ROS tutorial [Examining the Simple Publisher and Subscriber](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber).

Make sure that your roscore is up and running
```
roscore
```
Running roscore will tie up your terminal window, so you'll need to open a new terminal window.

In the new terminal window, since we are using Catkin, we'll want to make sure that we've sourced our workspace's setup.sh file:
```
cd ~/catkin_ws/
source ./devel/setup.bash
```

Next we will start our "talker" publisher:
```
rosrun beginner_tutorials talker
```

Running the talker will again tie up your terminal window, so we'll need to open an additional window to run the listener.

In the new terminal window (your third) source the workspace's setup.sh file again:
```
cd ~/catkin_ws/
source ./devel/setup.bash
```

Then begin running the listener:
```
rosrun beginner_tutorials listener
```

When you are done, press Ctrl-C to terminate the talker and the listener.
