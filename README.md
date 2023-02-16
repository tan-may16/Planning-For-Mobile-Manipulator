# Motion Planning-For-Mobile-Manipulator
16782- Planning and Decision Making project


##Overview

This repository contains implementation of motion planning for mobile manipulator in ROS and gazebo environment. The aim of the project is to implement constrained 3D A* algorithm for navigation and variations of RRT for manipulation.
The code assumes that ROS (Melodic/Neotic), gazebo as well as other basic dependencies are already installed.

##Installation and Implementation

**Step 1: Clone this repository.**

```shell
git clone https://github.com/tan-may16/Planning-For-Mobile-Manipulator.git
```

**Step 2: Install the project specific Dependencies (Replace distro with your ROS version name)**

```shell
sudo apt-get install libmove-base-msgs-dev
pip install rospkg
sudo apt-get install ros*controller*
sudo apt-get install ros-distro*joint-trajectory-controller*
sudo apt-get install mate-terminal
sudo apt-get install xdotool
```

**Step 3: Get the Gazebo Worlds.**

The repository uses the pre-developed gazebo worlds. These worlds can be downloaded using the following instructions.

1. Delete the existing gazebo_models_worlds_collection folder.

2. Download all the models and worlds from the gazebo world collections.

```shell
git clone https://github.com/leonhartyao/gazebo_models_worlds_collection.git
```

3. Follow the instructions provided in the gazebo_models_worlds_collection repository.In particular, do the following:

Open the bash file and export the path.

```shell
gedit ~/.bashrc
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<path to this repo>/models
export PYTHONPATH=$PYTHONPATH:/usr/lib/python3.8/dist-packages
```
Change the Python version in the last line if different. Save the bash file and close it. 


4. Edit the world file. A sample edited world file for office_smalll has been provided in the sample_world folder.

Note: To use the sample world, replace the office_small.world file from sample_world folder with the gazebo_models_worlds_collection/worlds/office_small.world file.

5. Make sure the world path is correct in the following mobile manipulator launch file: src/mobile_manipulator/launch/mobile_manipulator_gazebo.launch

Note: The path of the world is on line 9. Even if you are using sample world, you will have to edit this path.

**Step 4: Build ROS packages.**

Copy the src folder from the repository into your catkin workspace src folder (catkin_ws/src).

Build the packages
```shell
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

**Step4: Run the combinedPlanner.sh file with appropriate required arguments to visualize and test results.**
```shell
cd Planning-For-Mobile-Manipulator
./combinedPlanner.sh 2 3 180 2 -3 0 3.14 1.047 0.3 1.57 0
```
## Command Line Arguments

All the command line arguments are mandatory arguments.

First 3 arguments represent initial position of mobile manipulator(x_init, y_init, theta_init). 

Similarly,next 3 arguments represent final position of mobile manipulator (x_final, y_final, theta_final)

 Note: The above thetas are in degress.
 
 Last 5 arguments represent the final/desired configuration of manipulator when it reaches the destination. (In radians)
 
 
 
