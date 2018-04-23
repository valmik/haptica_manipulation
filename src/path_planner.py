#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory, RobotState, PlanningScene, CollisionObject
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from trac_ik_python.trac_ik import IK
from moveit_msgs.srv import GetPositionFK
from std_msgs.msg import Header

import sys
import numpy as np



class PathPlanner():
    """
    This class should contain all path planning functionality.
    The goal is that no ROS should be needed to interface with this node
    All ROS should be self-contained here
    """

    def __init__(self):
        """
        Shouldn't need inputs. Stores:
            2 move groups: arm and hand - moveit commander interface
            planning scene interface and publisher
            robot commander
            IK solver (using trac_ik)
            FK solver (moveit service)
        """

        rospy.loginfo("To stop project CTRL + C")
        rospy.on_shutdown(self.shutdown)

        # Initialize moveit_commander
        # I honestly don't think it actually uses the sys.argv argument,
        # but I need to do a little more digging first. It parses the input
        # for arguments containing "__name:=" which it passes to another initializer
        # I need to dig further to see what the name parameter actually changes
        moveit_commander.roscpp_initialize(sys.argv)

        # Instatiate the move group
        self.group = moveit_commander.MoveGroupCommander('arm')
        self.group.set_planning_time(5)
        self.group.set_workspace([-3, -3, -3, 3, 3, 3])

        # This publishes trajectories to RVIZ for visualization
        # Need to figure out how to get rid of the visualization as well
        # Also how to use this to publish individual poses
        self.display_planned_path_publisher = rospy.Publisher('arm/display_planned_path', DisplayTrajectory, queue_size=10)

        # This publishes updates to the planning scene
        self.planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

        # Planning scene
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # RobotCommander
        self.robot = moveit_commander.RobotCommander()

        # IK Solver with trac_ik
        # NOTE: Get this from the MoveGroup so it's adaptable to other robots
        self.ik_solver = IK('root', 'm1n6s300_end_effector')
        self.ik_default_seed = [0.0] * self.ik_solver.number_of_joints
        
        # FK Solver
        rospy.wait_for_service('/compute_fk')
        self.fk_solver = rospy.ServiceProxy('/compute_fk', GetPositionFK)

        rospy.sleep(2)        

        # Rate is 10 Hz. Is this a good rate?
        rate = rospy.Rate(10)


    def shutdown(self):
        """
        Things to do when the node is shut down
        (currently does nothing besides printing a message, 
        should it do anything?)
        """
        rospy.loginfo("Stopping project")
        rospy.sleep(1)


    def plan_to_config(self, end_state):
        """
        Plans to a configuration using MoveIt.
        Does not deal with finger positions (these stay static)
        -------
        end_state: list of joint angles, discluding fingers (length 6)
        -------
        returns: moveit path object (array of JointStates or RobotStates I believe)
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

    def plan_to_pose(self, end_pose, seed = None):
        """
        Plans to a pose by using inverse kinematics.
        -------
        end_pose: rigid_transform object from autolab? (or should I write my own? Or allow different inputs?) 
        seed: an optional input to the IK solver.
            It's a configuration (len 6 array) from which the solver will start. (trac_ik uses an augmented newton's method)
            If it's None, it'll use the current pose as the seed
        -------
        returns: moveit path object
        """
        raiseNotDefined()

    def move_home(self):
        """
        Plans from the current state to the home position
        -------
        returns: moveit path object
        """

        self.group.set_start_state_to_current_state()
        self.group.set_named_target('Home')

        self.group.set_workspace([-3, -3, -3, 3, 3, 3])

        plan = self.group.plan()

        return plan

    def execute_path(self, path, wait_bool = True):
        """
        Executes an input path in the real world / gazebo / rviz
        ------
        path: a moveit path object
        wait_bool: whether the node pauses until the robot reaches its destination
        """

        self.group.execute(path, wait=wait_bool)

    def get_ik(self, pose, seed = None, xyz_bounds = None, rpy_bounds = None):
        """
        Uses trac_ik to get a joint configuration from an input pose
        Does not include collision checking
        ------
        pose: rigid transform from autolab? See plan_to_pose
        seed: list of size 6. start value for the IK. See plan_to_pose. If None, will use current pose
        xyz_bounds: list of size 3. xyz bounds of the end effector in meters. This allows an approximate ik solution. Default is None
        rpy_bounds: a list of size 3. roll pitch yaw bounds of the end effector in radians. This allows an approximate ik solution. Default is None 
        ------
        returns: joint states, a list of size 6 (or None for failure)
        """

        if seed_state is None:
            seed_state = self.ik_default_seed

        if pose.header.frame_id != self.ik_solver.base_link:
            raise ValueError("Frame ID is not the root link")

        position = pose.pose.position
        orientation = pose.pose.orientation

        print seed_state, position.x, position.y, \
            position.z, orientation.x, orientation.y, \
            orientation.z, orientation.w \

        if xyz_bounds is None or rpy_bounds is None:
            state = self.ik_solver.get_ik(seed_state, position.x, 
                position.y, position.z, orientation.x, orientation.y, 
                orientation.z, orientation.w)
        else:
            state = self.ik_solver.get_ik(seed_state, position.x, 
                position.y, position.z, orientation.x, orientation.y, 
                orientation.z, orientation.w, xyz_bounds[0], 
                xyz_bounds[1], xyz_bounds[2], rpy_bounds[0], 
                rpy_bounds[1], rpy_bounds[2])

        return state

    def get_fk(self, joints):
        """
        Uses the MoveIt service to get the end effector pose from joints
        Note that this is the pose of the beginning of the hand, not the hand itself
        ------
        joints: a list of size 6. Joint states for the calculation
        ------
        returns: a rigid transform from autolab? See plan_to_pose
        """
        header = Header()
        header.frame_id = self.ik_solver.base_link

        robot_state = RobotState()
        robot_state.joint_state.name = self.ik_solver.joint_names
        robot_state.joint_state.position = joints

        links = [self.ik_solver.tip_link]

        return self.fk_solver(header, links, robot_state).pose_stamped[0]

    def collision_free(self, joints):
        """
        Checks if the configuration collides with anything in the planning scene
        ------
        joints: a list of size 6. Configuration for the collision check
        ------
        returns: boolean True if it's ok, False if it collides
        """
        raiseNotDefined()        

    def add_stl_to_scene(self, id, fileName, pose, scale):
        """
        Adds an STL object to the scene
        ------
        id: object id (string)
        fileName: location of the STL file
        pose: autolab rigid_transform?
        scale: len 3 list defining scale in x,y,z
        """
        raiseNotDefined()

    def add_box_to_scene(self, id, pose, size):
        """
        Adds a box to the scene
        ------
        id: object id (string)
        pose: see above
        size: len 3 list for xyz dimensions
        """
        raiseNotDefined()

    def populate_scene():
        """
        Not sure how this will be implemented yet, but will basically
            use the PathPlanner planning scene functionality to add
            the scene items needed for our test/demo
        """

    def remove_object_from_scene(self, id):
        """
        removes object from the scene
        ------
        id: object id
        """
        raiseNotDefined()

    def attach_object_to_robot(self, id):
        """
        attaches object to the robot
        ------
        id: object id
        """
        raiseNotDefined()

    def detach_object_from_robot(self, id):
        """
        detaches object from robot
        ------
        id: object id
        """
        raiseNotDefined()

    def visualize_plan(self, plan): # Untested
        """
        Visualize the plan in RViz
        """

        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = plan.points[0]
        display_trajectory.trajectory.extend(plan.points)
        self.display_planned_path_publisher.publish(display_trajectory)

    def visualize_state(self, joint_state):
        """
        Visualize a joint state in RViz
        """

        raiseNotDefined()

    def iterate_ik(self, pose, state_list = None, xyz_bounds = None, rpy_bounds = None, iteration_limit = 10):
        """
        Iterates IK until a non-colliding solution is found, or it hits the iteration limit
        ------
        pose: see get_ik
        state_list: list of length 6 lists, each of which is an initial condition to be tested.
            After the list is exhausted, the system will use pseudorandom start conditions
            until the iteration count runs out (hopefully it'll pick random states intelligently)
        xyz_bounds: see get_ik
        rpy_bounds: see get_ik
        iteration_limit: max number of initial conditions the algorithm will try (including the list)
            Default is 10.
        ------
        returns: joint states, a list of size 6 (or None for failure)
        """
        raiseNotDefined()


    def grasp_plan(self, pregrasp, grasp):
        """
        Plans to a pregrasp pose, then plans to the grasp pose
        Does not open/close fingers
        This should be the main function used, and will use many of the other functions
        It will (hopefully) select intelligent IK solutions
        ------
        pregrasp: An autolab rigid_transform? see plan_to_pose
        grasp: same as above
        ------
        returns: either two plans concatenated or just executes it
        """
        raiseNotDefined()




def raiseNotDefined():
    fileName = inspect.stack()[1][1]
    line = inspect.stack()[1][2]
    method = inspect.stack()[1][3]

    print "*** Method not implemented: %s at line %s of %s" % (method, line, fileName)
    sys.exit(1)




