# This is a skeleton for the path planning api I'll be building

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
        raiseNotDefined()


    def shutdown(self):
        """
        Things to do when the node is shut down
        (currently does nothing besides printing a message, 
        should it do anything?)
        """
        raiseNotDefined()

    def plan_to_config(self, end_state):
        """
        Plans to a configuration using MoveIt.
        Does not deal with finger positions (these stay static)
        -------
        end_state: list of joint states, discluding fingers (length 6)
        -------
        returns: moveit path object (array of JointStates or RobotStates I believe)
        """
        raiseNotDefined()

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
        raiseNotDefined()

    def plan_to_saved_config(self, save_point="home"):
        """
        Plans to one of the saved configs somewhere
        -------
        save_point is a string naming one of the saved configurations
        -------
        returns: moveit path object
        """
        raiseNotDefined()

    def close_grippers(self):
        """
        Plans and executes a gripper close
        """

    def open_grippers(self):
        """
        Plans and executes grippers opening
        """

    def execute_path(self, path, wait_bool = True):
        """
        Executes an input path in the real world / gazebo / rviz
        ------
        path: a moveit path object
        wait_bool: whether the node pauses until the robot reaches its destination
        """
        raiseNotDefined()

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
        raiseNotDefined()

    def get_fk(self, joints):
        """
        Uses the MoveIt service to get the end effector pose from joints
        Note that this is the pose of the beginning of the hand, not the hand itself
        ------
        joints: a list of size 6. Joint states for the calculation
        ------
        returns: a rigid transform from autolab? See plan_to_pose
        """
        raiseNotDefined()

    def collision_free(self, joints):
        """
        Checks if the configuration collides with anything in the planning scene
        ------
        joints: a list of size 6. Configuration for the collision check
        ------
        returns: boolean True if it's ok, False if it collides
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



def raiseNotDefined():
    fileName = inspect.stack()[1][1]
    line = inspect.stack()[1][2]
    method = inspect.stack()[1][3]

    print "*** Method not implemented: %s at line %s of %s" % (method, line, fileName)
    sys.exit(1)
