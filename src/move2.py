#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory, RobotState, PlanningScene, CollisionObject
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState

import sys
import numpy as np



class PathPlanner(object):
    def __init__(self):
        """
        Path Planner Class.
        Referenced dynamicreplanning.weebly.com and moveit python interface tutorial
        """
        rospy.loginfo("To stop project CTRL + C")
        rospy.on_shutdown(self.shutdown)

        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        # Instatiate the move group
        self.group = moveit_commander.MoveGroupCommander('arm')
        self.group.set_planning_time(5)
        self.group.set_workspace([-3, -3, -3, 3, 3, 3])

        # This publishes trajectories to RVIZ for visualization
        self.display_planned_path_publisher = rospy.Publisher('arm/display_planned_path', DisplayTrajectory, queue_size=10)

        # This publishes updates to the planning scene
        self.planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

        # Planning scene
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # RobotCommander
        self.robot = moveit_commander.RobotCommander()
        

        rospy.sleep(2)        

        rate = rospy.Rate(10)

    def shutdown(self):
        rospy.loginfo("Stopping project")
        rospy.sleep(1)

    def plan_to_config(self, end_state):
        """
        Uses MoveIt to plan a path from the current state to end_state and returns it
        end_state: list of 6 joint values
        """

        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        # the robot has a dumb base_link joint that we don't want
        joint_state.name = self.group.get_joints()[1:-1]
        print joint_state.name
        joint_state.position = end_state

        self.group.set_start_state_to_current_state()

        try:
            self.group.set_joint_value_target(joint_state)
        except moveit_commander.MoveItCommanderException:
            pass
        
        # Plan the path
        plan = self.group.plan()

        return plan

    def execute_path(path, wait_bool = True):
        """
        Executes a provided path.
        Note that the current position must be the same as the path's initial position.
        This is currently not checked.
        """

        self.group.execute(path, wait=wait_bool)

    # def collision_free_move_pose(self, end_pose):
    #     """
    #     Uses MoveIt to plan a path from the current state to end effector pose end_pose
    #     end_pose: a PoseStamped object for the end effector
    #     """

    #     self.group.set_start_state_to_current_state()
    #     self.group.set_joint_value_target(end_pose)

    #     self.group.set_workspace([-3, -3, -3, 3, 3, 3])

    #     plan = self.group.plan()

    #     return plan

    def move_home(self):
        """
        Uses MoveIt to plan a path from the current state to the home position
        """

        self.group.set_start_state_to_current_state()
        self.group.set_named_target('Home')

        self.group.set_workspace([-3, -3, -3, 3, 3, 3])

        plan = self.group.plan()

        return plan



    def visualize_plan(self, plan):
        """
        Visualize the plan in RViz
        """

        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = plan.points[0]
        display_trajectory.trajectory.extend(plan.points)
        self.display_planned_path_publisher.publish(display_trajectory)

def make_pose(position, orientation, frame):
    """
    Creates a PoseStamped message based on provided position and orientation
    position: list of size 3
    orientation: list of size 4 (quaternion)
    frame: string (the reference frame to which the pose is defined)
    """

    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]
    pose.pose.orientation.w = orientation[0]
    pose.pose.orientation.x = orientation[1]
    pose.pose.orientation.y = orientation[2]
    pose.pose.orientation.z = orientation[3]
    return pose

def add_arbitrary_obstacle(size, id, pose, planning_scene_publisher, scene, robot, operation):
    """
    Adds an arbitrary rectangular prism obstacle to the planning scene.
    This is currently only for the ground plane
    size: numpy array of size 3 (x,y,z dimensions)
    id: string (object id/name)
    pose: PoseStamped objecct (objects pose with respect to world frame)
    planning_scene_publisher: ros Publisher('/collision_object', CollisionObject)
    scene: PlanningSceneInterface
    robot: RobotCommander
    operation: currently just use CollisionObject.ADD
    """

    co = CollisionObject()
    co.operation = operation
    co.id = id
    co.header = pose.header
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = size
    co.primitives = [box]
    co.primitive_poses = [pose.pose]
    planning_scene_publisher.publish(co)



if __name__ == '__main__':
    rospy.init_node('kinova_controller')

    # if len(sys.argv) > 1:
    #     filename = sys.argv[1]

    joints1 = [4.80, 2.92, 1.00, 4.20, 1.45, 1.32]
    joints2 = [0.0, 2.9, 1.3, 4.2, 1.4, 0.0]

    path_planner = PathPlanner()

    raw_input("Press Enter to move to home")
    plan = path_planner.move_home()
    path_planner.execute_path(plan)
    rospy.sleep(0.5)

    while True:
        raw_input("Press Enter to move to position 1")
        plan = path_planner.plan_to_config(joints1)
        path_planner.execute_path(plan)
        rospy.sleep(0.5)

        raw_input("Press Enter to move to position 2")
        plan = path_planner.plan_to_config(joints2)
        path_planner.execute_path(plan)
        rospy.sleep(0.5)






