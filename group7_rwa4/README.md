This is README to run the ROS package created for RWA4. 

## **Dependencies**
  * ROS Melodic 
  * Gazebo 9.6 
  * Ariac 2019

-make file by (or can use --only option)
catkin_make

-To launch the environment, open a terminal and run
roslaunch group7_rwa4 group7_rwa4.launch

-open two different terminals and run
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm2

-open the 4th terminal and run
rosrun group7_rwa4 main_node

-open 5th terminal and run
rosservice call /ariac/start_competition 
