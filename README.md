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
