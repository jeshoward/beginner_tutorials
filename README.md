# ENPM808X - ROS Services, Logging, and Launch Files
This is the continuation of the implementation of several ROS beginner tutorials:
1. [Creating a Workspace for Catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
2. [Creating a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
3. [Building a ROS Package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)
4. [Creating a ROS msg and srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
5. [Writing a Simple Publisher and Subscriber (C++)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

The original goal of the project was to install ROS and Catkin, create a workspace, and create a simple publisher and subscriber package. This branch (Week10_HW) continues the effort by applying lessons learned in the following intermediate ROS tutorials:
1. [Getting started with roswtf](http://wiki.ros.org/ROS/Tutorials/Getting%20started%20with%20roswtf)
2. [Understanding ROS Services and Parameters](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)
3. [Using rqt_console and roslaunch](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch)

Additional services added to the beginner tutorials are: Add Two Ints

The Add Two Ints service was implemented as per the tutorial [Writing a Simple Service and Client](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29). 

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
git clone -b Week10_HW https://github.com/jeshoward/beginner_tutorials
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

Next we will start our launcher, which begins the addition and multiplication servers:
```
roslaunch beginner_tutorials week10_hw.launch
```

Open a new terminal window to run the addition or multiplication services. 

#### Add Two Ints
Add two ints takes two integer arguments, a and b, and adds them together.
```
rosrun beginner_tutorials add_two_ints_client a b
```

You can also input the arguments for a and b from the launcher by using the following code:
```
roslaunch beginner_tutorials week10_hw.launch a:=1 b:=2
```

When you are done, press Ctrl-C to terminate the servers.

## License
BSD 3-Clause License

Copyright (c) 2017, Jessica Howard
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
