# Onboarding RoboSub
This branch is in charge of onboarding new members with ROS knowledge and the basics of the software stack for Robosub. This tutorial can also be used as a general onboard for other teams such as RobotX, Microtransit, and RoboBoat. 
It is **highly recommended** (if not required) for new members to complete this onboarding process before attempting any tasks. It is also highly recommended to truthfully go through this onboarding process without 
directly going to the answers. This is for YOU, so if you want to skip around to the more advanced topics or topics you are not familar with, go for it as long as you end this onboarding process with the knowledge to complete tasks. 

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

Then go to your workspace directory and run:
```
colcon build
source install/setup.bash
```

## Topic 1: ROS2 Basics
At any point you want to test your code, you will need to run these commands in your workspace directory.
```
colcon build
source install/setup.bash
ros2 run onboarding test_topic_{topic_number}_{topic_subsection}
```
For example, to run the tests for section 1.2, you should run `ros2 run onboarding test_topic_1_2`
### 1.1 Understanding Nodes and Topics
The goal of this section is to familiarize yourself with the concept of nodes and topics. We will be using ROS2 CLI throughout this section and you will be filling out answers in the file `/mrg_ws/src/Pontus/onboarding/onboarding/questions/topic_1/question1_1.py`. 

#### 1.1.a Nodes
First in your terminal, run `ros2 node list`. This will list all running nodes. You should currently have 0 running nodes. Now, in the terminal, run `ros2 run onboarding node_q_1_1`. Change the value of `num_nodes` to the new number of nodes. 

#### 1.1.b Node Names
Change the value of `first_node_name` to the name of the first node. 

Note: When we ran the node, we used the command `ros2 run onboarding node_q_1_1`. As you can see the name `node_q_1_1` will not always 
match the name of the node. Here, `node_q_1_1` refers to the executable name, which is defined in `setup.py`.

#### 1.1.c Topics
Stop the node. This can be done by going into the terminal where you ran the node and press `CTRL + c`. Once you have killed the node,
run `ros2 topic list`. You should see two topics, namely `/parameter_events` and `rosout`. These are system-generated topics and you do not have to worry about these topics for now. Again, run `ros2 run onboarding node_q_1_1`. Change the value of `num_topics` to the new number of topics. 

#### 1.1.d Topic Info
Sometimes you may want to find information about a topic. You can use `ros2 topic info TOPIC_NAME` to do so. First, run `ros2 topic info /onboarding/OdometryPub`. Note the fields `Type`, `Publisher count`, and `Subscription count`. One of the other topics is called `/onboarding/MysterPub`. We want to find what type of message this topic publishes. Change the value of `topic_message_type` to the correct message type of the topic `/onboarding/MysterPub`. Write your answer as a string.

#### 1.1.e Topic Echo
When debugging, it sometimes important to checkout what messages are being published to a topic. A useful command for this is `ros2 topic echo TOPIC_NAME`. Change the value of `string_message` to the string that is being published to the topic `/onboarding/StringPub`. 

### 1.2 Coding Subscriber and Publishers
The goal of this section is to understand what a publisher and subscriber is within ROS2 and how to create them. Please look at file `topic1.py` in `/mrg_ws/src/Pontus/onboarding/onboarding/questions`.
After completing the questions, to run your code you will need to run these commands in your workspace directory:
