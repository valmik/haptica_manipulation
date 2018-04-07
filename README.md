# haptica_manipulation

Runs MoveIt in Gazebo

run:
roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=m1n6s300

roslaunch m1n6s300_moveit_config m1n6s300_gazebo_demo.launch

rosrun haptica_manipulation move_robot.py








[//]: # "catkin_create_pkg haptica_manipulation pluginlib moveit_core moveit_ros_planning_interface moveit_ros_perception interactive_markers cmake_modules geometric_shapes kinova_driver moveit_fake_controller_manager std_msgs moveit_msgs geometry_msgs shape_msgs"


