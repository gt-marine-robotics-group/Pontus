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
Each section has an associated autograder. At any point you want to test your code, you will need to run these commands in your workspace directory.
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
Sometimes you may want to find information about a topic. You can use `ros2 topic info TOPIC_NAME` to do so. First, run `ros2 topic info /onboarding/OdometryPub`. Note the fields `Type`, `Publisher count`, and `Subscription count`. One of the other topics is called `/onboarding/MysteryPub`. We want to find what type of message this topic publishes. Change the value of `topic_message_type` to the correct message type of the topic `/onboarding/MysteryPub`. Write your answer as a string.

#### 1.1.e Topic Echo
When debugging, it sometimes important to checkout what messages are being published to a topic. A useful command for this is `ros2 topic echo TOPIC_NAME`. Change the value of `string_message` to the string that is being published to the topic `/onboarding/StringPub`. 

### 1.2 Coding Subscriber and Publishers
The goal of this section is to understand what a publisher and subscriber is within ROS2 and how to create them. Please look at file `question1_2.py`

#### 1.2.a Creating a subscriber to a topic
Creating a subscriber is a way to access data that is sent over a topic. For this question, fill in the blanks to create a subscriber.
This subscriber should
- Subscribe to the topic `/onboarding/basic_topic`
- Have its callback be `self.topic_callback`
- Take in a message type `String`
- Qos profile of 10

HINT: There should be four parameters that you fill

#### 1.2.b Creating a publisher
Creating a publisher is a way to send data over a topic. For this question, fill in the blanks to create a publisher.
This publisher should
- Publish a message type `String`
- Publish to a topic `/onboarding/new_topic`
- Qos profile of 10

HINT: There should be three parameters that you fill

#### 1.2.c Message Data
Please see what the String message type consists of: https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html.
For this question, access the string field from the variable `msg` and store it into the variable `self.topic_string_message`

#### 1.2.d Publishing Data
Publishing data is how we send information to a topic. For this question, take the value from `topic_string_message` and append the string ` ROS`. The new string should look like `Hello World! ROS`. Then use the variable `new_message` to publish this new string using the publisher from **Q1.1.b**.

### 1.3 Counter Node
This question is designed to test your knowledge on this topic. Take a look at `question1_3.py`. The goal of this node is to publish to a topic called `/onboarding/counter250` with numbers of type `Int32` starting from 0 incrementing to 250 inclusive. A rough outline has been provided for you. Fill in the blanks to complete this question. 

## Topic 2: Coordinate Frames

This section we will be going over coordinate frame standards in ros. For more information, see [REP - 103: Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html). Coordinate frames are a way of defining points in space. An example would be your standard xy-coordinate frame.  
![image](https://github.com/user-attachments/assets/a1a3b214-a865-42ab-8f38-468dbd836b19)

### 2.1 ENU Convention

In ROS, we have a set of standard coordinate frames defined by the right-hand rule. We will first consider the ENU convention (east(x) - north(y) - up(z)), which would look something like this:

<img src="https://github.com/user-attachments/assets/b4d2da4b-1d2f-432f-98ae-e85586060eb4" alt="image" width="400">

This means the following relationships:
- The more east a point is, the more positive its x
- The more north a point is, the more positive its y
- The higher a point is, the more positive its z

For this question, fill in the three variables with the correct values based on the following image and the ENU convention. You can assume that the axes represent true north and true east. 

<img src="https://github.com/user-attachments/assets/7c5b05f6-f29b-4ed8-9dec-b0be8514f349" alt="image" width="400">

### 2.2 NED Convention

The NED convention is north(x) - east(y) - down(z), which looks something like:

<img src="https://github.com/user-attachments/assets/bf7a847e-13ba-4aca-937c-46e30ed8462a" alt="image" width="300">

For this question, fill in the three variables with the correct values based on the following image and the NED convention. You can assume that the axes represent true north and true east. 

<img src="https://github.com/user-attachments/assets/7c5b05f6-f29b-4ed8-9dec-b0be8514f349" alt="image" width="400">


## Topic 3: Understanding Odometry

### 3.1 Understanding Pose
Take a look at [Pose Message](https://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/Pose.html). The pose message contains two fields, a **position** and an **orientation**. This message type is used to define the location and facing of some object in some world space. For example, where our sub is and what heading we are facing.

Understanding Position
Create a subscriber that subscribes to the topic `/onboarding/pose` with message type `Pose`. In the callback function, assign the values `self.position_x`, `self.position_y`, and `self.position_z` to the respective values in the `Pose` message.

## Topic 4: Simulation + Useful Tools

## Topic 5: Perception

## Topic 6: Sonars and Point Clouds

## Topic 7: Autonomy
