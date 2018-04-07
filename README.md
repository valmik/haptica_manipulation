# haptica_manipulation

Runs MoveIt in Gazebo

run:
roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=m1n6s300

roslaunch m1n6s300_moveit_config m1n6s300_gazebo_demo.launch

rosrun haptica_manipulation move_robot.py

