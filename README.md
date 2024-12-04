# Onboarding RoboSub
This branch is in charge of onboarding new members with ROS knowledge and the basics of the software stack for Robosub. This tutorial can also be used as a general onboard for other teams such as RobotX, Microtransit, and RoboBoat. 
It is **highly recommended** (if not required) for new members to complete this onboarding process before attempting any tasks. It is also highly recommended to truthfully go through this onboarding process without 
directly going to the answers.

## Prerequisites 
- **Some base knowledge of coding/python** If you are not yet familiar with python, you can still attempt to go through this onboarding process but it is highly recommended that you learn python first. Some great tutorials are:
  - https://www.w3schools.com/python/
  - https://docs.python.org/3/tutorial/index.html
- **ROS2 Installation** We recommend using a docker image for all your work. We have created a docker image for you that will have ROS2 pre-installed along with a bunch of dependencies already downloaded for you. 
The link to the docker image is: https://github.com/chachmu/MRG_Docker. Please follow the instructions in the github readme to get it setup. Please do not be afraid to contact any of the software leads for help!
- **Pontus repo cloned** If you are using the docker image, you should have already set this up when setting up the docker image. However, if you decided to not use the docker image, you will need to make sure 
to clone this repository into your ROS workspace. A suggested folder structure is the following:
```
mrg_ws/
  |__ src/
       |__ Pontus/
```
This tutorial will assume that you have the following file structure and **will refer to `mrg_ws` as the workspace directory**. To clone the repo, first ensure that you are in the correct directory. To verify this, you can run the command `pwd` in your terminal. 
You should see something like: `/home/name/mrg_ws/src`. If you are in the incorrect directory, you can run the command `ls` in ther terminal to list the current directories in your folder. For example, after `ls` you might see something like the following:
```
mrg_ws
Downloads
Documents
Desktop
```
To change directories, you can run `cd DIRECTORY_NAME`. So in the above example, to get into the `mrg_ws` directory, one should run
```
cd mrg_ws
```

After you ensure that you are in the correct directory run:
```
git clone https://github.com/gt-marine-robotics-group/Pontus.git
```

## Setup
First ensure that you are on the correct branch of the Pontus repository. If your run `git branch`, in the `/mrg_ws/src/Pontus` directory, you should see something like the following:
```
  main
* onboarding
```
The `*` should be next to onboarding. If it is not, run `git checkout onboarding`

## Topic 1: ROS2 Basics
### 1.1 Coding Subscriber and Publishers
The goal of this section is to understand what a publisher and subscriber is within ROS2 and how to create them. Please look at file `topic1.py` in `/mrg_ws/src/Pontus/onboarding/onboarding/questions`.

After completing the questions, to run your code you will need to run these commands in your workspace directory:
```
colcon build
source install/setup.bash
ros2 run onboarding test_topic_1
```

