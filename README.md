# follow-path-points

This is a tutorial on how to execute the codes in this repository.

This is a work that uses ROS - it is very recommended that the user installs the accordingly ROS Distro for their system. In case of using Ubuntu, every LTS version of Ubuntu has a ROS Distro for it, up until 20.04. Since this work uses ROS1, and ROS1 was discontinued in 2020, 20.04 is the last LTS version that has a ROS1 Distro, ROS Noetic. The other ones (22.04 and further) all have ROS2 Distros. For more information on how to install ROS, you can follow <a href="http://wiki.ros.org/ROS/Installation">this tutorial here.</a>

This work makes use of a lot of features already present in ROS - that is, the Navigation Stack, TurtleBot3, DWA, AMCL, RViz, Gazebo. It is recommended their packages are also installed. To be sure these packages are properly installed, please execute the following commands:

```
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-gmapping
sudo apt-get install ros-kinetic-amcl
```

A catkin workspace is also necessary: follow <a href="http://wiki.ros.org/catkin/Tutorials/create_a_workspace">this tutorial</a> to make sure one is properly set up. In there, clone this repository in the src folder, and, in the root of the catkin workspace, execute the following command:

```
catkin_make --only-pkg-with-deps follow-path-points
```

This will make the compilation of this - and only this - workspace and make it ready to be executed.

After this, this project is ready to be executed. For this, two archives need to be launched: a .launch file that reunites all the necessary things for the project to run, and a python script that sends the goals and routes to the robot. The launch script - <i>nav_stack.launch</i> - reunites a few things. 

First of all, <i>move_base</i>, that is responsible for making all the planners work. They'll read the according parameters to each planner - Global and Local planners and costmaps - and be able to make the correct trajectory with the use of Djikstra. The costmap parameters controls the execution of the map in the global - that is, the whole ambient - and local - what the robot can see right now with its sensors and liDARs - scale. The planner paramenters are responsible for the path planning, with the use of GlobalPlanner and DWA.

The launch file also launches <i>AMCL</i> - Adaptive Monte Carlo Localization - to be able to locate the robot in the given space, given its surroundings and odometry readings. Considering the robot is in a virtual space and has little margin for error, it's very hard that the robot will lose itself and give an innacurate position as a result.

It also launches a map server script, on which it reads the map of the ambient it's navigating on - in the case of this simulation, it's just an empty world - and its YAML file, with the informations like the size of the map and the pixels/meter rate that it considers inside.

And, last, but not least, a python script that reads the <i>TF</i> transformations made by ROS and publishes the according ones in a ROS Topic. That is, by reading the TFs, the script is able to know the precise localization of the robot in the virtual space, in x/y coordinates, and by publishing it in a topic, the main script is able to read and interpret it accordingly later.

There's also a .json file, <i>route.json</i>. This file is a list with all the points the robot needs to go through. Before the execution of the scripts, any point, any x/y coordinate, can be added or removed accordingly, as needed.

The main code, <i>main.py</i>, is not a part of the nav_stack launch. It reads the according information from the .json file, so it knows the points it needs to go to, it reads the according information from the topic the TF transformation is publishing on, so it always knows the precise localization of itself in the given space, and so it passes the points and goals to the topic where it should be read to send the coordinates of each goal to the robot. As soon as the TF transformations can determine the robot is within the error margin of the goal (that is, a circle of 0.2 radius around the goal), it sends the next goal for the robot to move to.

To execute both of these codes and make the robot come to life, run these lines, each of them in a different terminal:

```
Terminal 1:
roslaunch follow-path-points nav_stack.launch
```

```
Terminal 2:
rosrun follow-path-points main
```

However, this needs to be simulated. For this, what is used is a Turtlebot3 robot in a gazebo/RViz interface. The according tutorial to make a Turtlebot3 robot be able to be simulated is <a href="https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/">here</a> - be sure to have the necessary Turtlebot3 packages and installed on the /src folder of the catkin workspace -, but it can be resumed in these commands, that need to be executed in two different terminals:

```
Terminal 3:
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

```
Terminal 4:
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```

With this, the robot should come to life, in RViz and Gazebo. A video showcasing all of this can be seen in <a href="https://www.youtube.com/watch?v=DPeQbkY3bgY&ab_channel=felizpefabricio">this link,</a> where the robot move through the points (0,0); (0,2); (2.8,4.8); (4.8;4.8), (4.8,2).

Thank you for reading! Please contact me if you have any doubts!
