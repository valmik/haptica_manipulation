# haptica_manipulation

Runs MoveIt in Gazebo

run:
roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=m1n6s300

roslaunch m1n6s300_moveit_config m1n6s300_gazebo_demo.launch

rosrun haptica_manipulation path_planner.py


# Issues:
- Update IK solver links to come from the move group rather than being manually entered
- Make an IK-based planner
    - collision checks on IK
    - check multiple IK solutions
    - intelligent initial conditions for kinematics
- Add a second move group for the gripper




# Valmik's Notes Below: Please Ignore


# Create Package
- need to update with: sensor_msgs, trac_ik

catkin_create_pkg haptica_manipulation pluginlib moveit_core moveit_ros_planning_interface moveit_ros_perception interactive_markers cmake_modules geometric_shapes kinova_driver moveit_fake_controller_manager std_msgs moveit_msgs geometry_msgs shape_msgs


# set_joint_value_target Issue

MoveGroupCommander.set_joint_value_target doesn't work for a list of joint values or a JointState message. Bounds error

the python function is a wrapper for this:

    bool moveit::planning_interface::MoveGroupInterface::setJointValueTarget(const sensor_msgs::JointState& state)
    {
      impl_->setTargetType(JOINT);
      impl_->getJointStateTarget().setVariableValues(state);
      return impl_->getJointStateTarget().satisfiesBounds(impl_->getGoalJointTolerance());
    }

impl_ is a MoveGroupInterfaceIMPL. getJointStateTarget returns a RobotState object, and getGoalJointTolerance returns a double

The RobotState function is (in header file not actual):

    bool satisfiesBounds(const JointModel* joint, double margin = 0.0) const
    {
    return satisfiesPositionBounds(joint, margin) && (!has_velocity_ || satisfiesVelocityBounds(joint, margin));
    }
    bool satisfiesPositionBounds(const JointModel* joint, double margin = 0.0) const
    {
    return joint->satisfiesPositionBounds(getJointPositions(joint), margin);
    }

This is calling the JointModel, which is inside moveit_core::robot_model. The actual model seems to be a revolute joint: Here's the function

    bool RevoluteJointModel::satisfiesPositionBounds(const double* values, const Bounds& bounds, double margin) const
    {
        if (values[0] < bounds[0].min_position_ - margin || values[0] > bounds[0].max_position_ + margin)
            return false;
        return true;
    }

Margin = GoalJointTolerance is instantiated at like 10^-4, so it's not a problem. What's happening is that the bounds are failing when they shouldn't. There's no error in the C++ implementation, so they don't realize / it doesn't matter, but with Python it throws an exception. Just catch it and it'll be fine.






