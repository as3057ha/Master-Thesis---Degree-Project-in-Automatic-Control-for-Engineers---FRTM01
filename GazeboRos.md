# How to setup Gazebo and Ros

To setup Gazebo with ROS you have to install ROS, Gazebo and a package that facilitates communication inbetween them called ros-gz.

There are different distributions of ROS and Gazebo, one can read about them and their compatability [here](https://gazebosim.org/docs/latest/ros_installation).

As of writing it is recommended to use ROS 2 Humble (LTS) together with Gazebo Fortress (LTS).


## Installing ROS


The install instructions for ROS 2 Humble can be found [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

## Installing Gazebo

The install instruction for Gazebo Fortress can be found [here](https://gazebosim.org/docs)

## Installing the compatability package ros-gz

Install instruction can be found [here](https://gazebosim.org/docs/latest/ros_installation).

# Descriptions of universal robots 
[Descriptions of universal robots ](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)

# How to run Gazebo

Instructions on running Gazebo can be found [here](https://gazebosim.org/docs/all/getstarted). But essentially you create sdf files which you then run with Gazebo.
To run a sdf file you use > ign gazebo randomFile.sdf.

# An example of a Gazebo Simulation it the URE5

[Pick and Place with URE5 in Gazebo](https://github.com/lihuang3/ur5_ROS-Gazebo)

# Youtube tutorials with ROS and Gazebo

- [Matrix creature in gazebo](https://www.youtube.com/watch?v=ayp87SjrwPc&ab_channel=SwagatKumar)

## Youtube tutorial on how to run a UR5-Gazebo simulation ( using ROS 1 sadly)

- [Universal robot simulation with moveit in gazebo](https://www.youtube.com/watch?v=ayp87SjrwPc&ab_channel=SwagatKumar)

### Links mentioned in the tutorial as refrences
- https://ros-planning.github.io/moveit_tutorials/
- https://github.com/ros-industrial/universal_robot
- https://github.com/lihuang3/ur5_ROS-Gazebo
- https://wiki.ros.org/urdf/XML/link
- https://sir.upc.edu/projects/rostutorials/10-gazebo_control_tutorial/index.html
- https://www.theconstructsim.com/control-gazebo-simulated-robot-moveit-video-answer/
- http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html
