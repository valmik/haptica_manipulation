#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory, RobotState, PlanningScene, CollisionObject
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState

import tf

import sys
import numpy as np



class KinovaController(object):
    def __init__(self):
        """
        Robot controller class.
        Referenced dynamicreplanning.weebly.com and moveit python interface tutorial
        """
        rospy.loginfo("To stop project CTRL + C")
        rospy.on_shutdown(self.shutdown)

        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        # Instatiate the move group
        self.group = moveit_commander.MoveGroupCommander('arm')
        self.group.set_planning_time(5)

        # This publishes trajectories to RVIZ for visualization
        self.display_planned_path_publisher = rospy.Publisher('arm/display_planned_path', DisplayTrajectory, queue_size=10)
        
        rate = rospy.Rate(10)

    def shutdown(self):
        rospy.loginfo("Stopping project")
        rospy.sleep(1)

    def collision_free_move(self, end_state):
        """
        Uses MoveIt to plan a path from the current state to end_state and returns it
        end_state: list of joint values
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
        except MoveItCommanderException:
            pass

        self.group.set_workspace([-3, -3, -3, 3, 3, 3])
        
        # Plan the path
        plan = self.group.plan()

        return plan

    def collision_free_move_pose(self, end_pose):
        """
        Uses MoveIt to plan a path from the current state to end effector pose end_pose
        end_pose: a PoseStamped object for the end effector
        """

        self.group.set_start_state_to_current_state()
        self.group.set_joint_value_target(end_pose)

        self.group.set_workspace([-3, -3, -3, 3, 3, 3])

        plan = self.group.plan()

        return plan

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


    planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject)

    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    rospy.sleep(2)

    kinova_controller = KinovaController()

    # print kinova_controller.group.get_current_joint_values()
    # print robot.get_current_state()

    pose = make_pose([0.25, 0.25, 0.25], [1, 0, 0, 0], robot.get_planning_frame())

    raw_input("Press Enter to move to home")
    plan = kinova_controller.move_home()
    kinova_controller.group.execute(plan, wait=True)
    rospy.sleep(0.5)

    # raw_input("Press Enter to move to position 1")
    # plan = kinova_controller.collision_free_move_pose(pose)
    # kinova_controller.group.execute(plan, wait=True)
    # rospy.sleep(0.5)

    while True:
        raw_input("Press Enter to move to position 1")
        plan = kinova_controller.collision_free_move(joints1)
        kinova_controller.group.execute(plan, wait=True)
        rospy.sleep(0.5)

        raw_input("Press Enter to move to position 2")
        plan = kinova_controller.collision_free_move(joints2)
        kinova_controller.group.execute(plan, wait=True)
        rospy.sleep(0.5)






