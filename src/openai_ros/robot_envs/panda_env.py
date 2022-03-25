from openai_ros import robot_gazebo_env
import rospy
from sensor_msgs.msg import JointState
import moveit_commander
import sys
import geometry_msgs.msg
import numpy as np

class PandaEnv(robot_gazebo_env.RobotGazeboEnv):
    """
    Superclass for all Panda environments.
    This class contain all the ROS functionalities that your robot will need 
    in order to be controlled. This class also checks that every ROS stuff 
    required is up and running.
    """

    def __init__(self):
        """Initializes a new Panda environment.
        """
        rospy.logdebug("Entered Panda Env")
        # Variables that we give through the constructor.

        # Internal Vars
        self.controllers_list = ["/effort_joint_trajectory_controller"] # "/franka_state_controller", 

        self.robot_name_space = "panda" # not sure

        self.reset_controls = False
        
        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(PandaEnv, self).__init__(controllers_list=self.controllers_list,
                                                robot_name_space=self.robot_name_space,
                                                reset_controls=False)
        
        # We Start all the ROS related Subscribers and publishers
        self.JOINT_STATES_SUBSCRIBER = '/joint_states'
        self.joint_names = ["panda_joint1",
                          "panda_joint2",
                          "panda_joint3",
                          "panda_joint4",
                          "panda_joint5",
                          "panda_joint6",
                          "panda_joint7"]
        
        self.gazebo.unpauseSim()
        self._check_all_systems_ready()

        self.joint_states_sub = rospy.Subscriber(self.JOINT_STATES_SUBSCRIBER, JointState, self.joints_callback)
        self.joints = JointState()
        
        # Start Services
        self.move_reach_object = MoveReach()
        
        # Wait until it has reached its Startup Position. (IS THIS NEEDED?)
        # self.wait_reach_ready()
        

        self.gazebo.pauseSim()
        # Variables that we give through the constructor.
        rospy.logdebug("init Panda Env DONE")


    # RobotGazeboEnv Virtual Methods
    # ----------------------------
    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        # TODO
        self._check_all_sensors_ready()
        rospy.logdebug("ALL SYSTEMS READY")
        return True
    # ----------------------------

    # Methods RobotEnv will need.
    # ----------------------------
    def _check_all_sensors_ready(self):
        self._check_joint_states_ready()
        rospy.logdebug("ALL SENSORS READY")
    
    def _check_joint_states_ready(self):
        self.joints = None
        while self.joints is None and not rospy.is_shutdown():
            try:
                self.joints = rospy.wait_for_message(self.JOINT_STATES_SUBSCRIBER, JointState, timeout=1.0)
                rospy.logdebug("Current "+str(self.JOINT_STATES_SUBSCRIBER)+" READY, joint state msg =>" + str(self.joints))
                #######
                #[DEBUG] [1647471371.616565, 0.037000]: Current /joint_states READY=>header: 
                #  seq: 1
                #  stamp: 
                #    secs: 0
                #    nsecs:  35000000
                #  frame_id: ''
                #name: 
                #  - panda_joint1
                #  - panda_joint2
                #  - panda_joint3
                #  - panda_joint4
                #  - panda_joint5
                #  - panda_joint6
                #  - panda_joint7
                #  - panda_finger_joint1
                #  - panda_finger_joint2
                #position: [-0.02021035531186932, -0.1780145835390119, -0.039891823013486594, -0.39966030614240644, 0.13088714986954653, 0.3093085265609563, 0.1273327860054314, 0.003549732266365417, 0.003549732266365417]
                #velocity: [-0.010641281198307822, -0.005648077897238885, 0.0027730053015947997, -0.0033547844904800094, 0.0345557937451551, 0.013926294769580575, 0.04140939919748013, 0.3164884045186821, 0.3164884045186821]
                #effort: [0.00022501934131722166, 2.6700870269835546, 0.14284200898672444, -1.2467790780663113, -0.23962544446798287, -2.472763751677799, 0.0009737094915219322, -4.567237077711568e-05, -4.567237077711568e-05]
                #######
            except:
                rospy.logerr("Current "+str(self.JOINT_STATES_SUBSCRIBER)+" not ready yet, retrying....")
        return self.joints

    def joints_callback(self, data):
        self.joints = data

    def get_joints(self):
        return self.joints
        
    def get_joint_names(self):
        return self.joints.name

    def set_trajectory_ee(self, action):
        """
        Sets the Pose of the EndEffector based on the action variable.
        The action variable contains the position and orientation of the EndEffector.
        See create_action
        """
        # Set up a trajectory message to PUBLISH.
        ee_target = geometry_msgs.msg.Pose()
        ee_target.orientation.w = 1.0
        ee_target.position.x = action[0]
        ee_target.position.y = action[1]
        ee_target.position.z = action[2]
        
        rospy.logdebug("Set Trajectory EE...START...POSITION="+str(ee_target.position))
        result = self.move_reach_object.ee_traj(ee_target)
        rospy.logdebug("Set Trajectory EE...END...RESULT="+str(result))
        
        return result
        
    def set_trajectory_joints(self, initial_qpos):
        """
        set the desired position of each joint, and execute the motion plan.
        """
        positions_array = [None] * 7
        positions_array[0] = initial_qpos["panda_joint1"]
        positions_array[1] = initial_qpos["panda_joint2"]
        positions_array[2] = initial_qpos["panda_joint3"]
        positions_array[3] = initial_qpos["panda_joint4"]
        positions_array[4] = initial_qpos["panda_joint5"]
        positions_array[5] = initial_qpos["panda_joint6"]
        positions_array[6] = initial_qpos["panda_joint7"]
 
        self.move_reach_object.joint_traj(positions_array)
        
        return True
        
    def create_action(self,position,orientation):
        """
        position = [x,y,z]
        orientation= [x,y,z,w]
        """
        gripper_target = np.array(position)
        gripper_rotation = np.array(orientation)
        action = np.concatenate([gripper_target, gripper_rotation])
        
        return action
    
    def create_joints_dict(self,joints_positions):
        """
        Based on the Order of the positions, they will be assigned to its joint name
        names_in_order:
        'joint1': 0.0,
        'joint2': 0.0,
        'joint3': 0.0,
        'joint4': -1.57079632679,
        'joint5': 0.0,
        'joint6': 1.57079632679,
        'joint7': 0.785398163397,
        """
        assert len(joints_positions) == len(self.joint_names), "Wrong number of joints, there should be "+str(len(self.join_names))
        joints_dict = dict(zip(self.joint_names,joints_positions))
        
        return joints_dict
        
    def get_ee_pose(self):
        """
        Returns geometry_msgs/PoseStamped
            std_msgs/Header header
              uint32 seq
              time stamp
              string frame_id
            geometry_msgs/Pose pose
              geometry_msgs/Point position
                float64 x
                float64 y
                float64 z
              geometry_msgs/Quaternion orientation
                float64 x
                float64 y
                float64 z
                float64 w
        """
        #self.gazebo.unpauseSim()
        gripper_pose = self.move_reach_object.ee_pose()
        #self.gazebo.pauseSim()
        return gripper_pose
        
    def get_ee_rpy(self):
        gripper_rpy = self.move_reach_object.ee_rpy()
        
        return gripper_rpy
        
    def wait_reach_ready(self):
        """
        # TODO: Make it wait for the init pose.
        """
        import time
        for i in range(20):
            current_joints = self.get_joints()
            joint_pos = current_joints.position
            print("JOINTS POS NOW="+str(joint_pos))
            print("WAITING..."+str(i))
            time.sleep(1.0)
            
        print("WAITING...DONE")
    # ----------------------------

    # Methods the TrainingEnvironment will need to define.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()
    
    
    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations):
        """
        Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """
        Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        """
        Get the current observation.
        """
        raise NotImplementedError()

    def _is_done(self, observations):
        """
        Checks if episode done based on observations given.
        """
        raise NotImplementedError()
    # ----------------------------
   
        
# Class that fulfills the Reach movement with MoveIt.
# ----------------------------
class MoveReach(object):
        # moveit_commander: python interfaces to moveit.
    def __init__(self):
        
        rospy.logdebug("In MoveReach init...")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.logdebug("moveit_commander initialised...")
        
        rospy.logdebug("Starting Robot Commander...")
        self.robot = moveit_commander.RobotCommander()
        #######
        #[ WARN] [1647471371.629620353]: Skipping virtual joint 'virtual_joint' because its child frame 'panda_link0' does not match the URDF frame 'world'
        #######
        rospy.logdebug("Starting Robot Commander...DONE")
        
        self.scene = moveit_commander.PlanningSceneInterface()  
        rospy.logdebug("PlanningSceneInterface initialised...DONE")
        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        rospy.logdebug("MoveGroupCommander for panda_arm initialised...DONE")

        
    def ee_traj(self, pose):
        
        self.group.set_pose_target(pose)
        result = self.execute_trajectory()
        
        return result
        
    def joint_traj(self, positions_array):
        """
        set target and excute the motion plan.
        """
        self.group_variable_values = self.group.get_current_joint_values()
        rospy.logdebug("Current Group Vars:")
        rospy.logdebug(self.group_variable_values)
        rospy.logdebug("Desired Point:")
        rospy.logdebug(positions_array)
        self.group_variable_values[0] = positions_array[0]
        self.group_variable_values[1] = positions_array[1]
        self.group_variable_values[2] = positions_array[2]
        self.group_variable_values[3] = positions_array[3]
        self.group_variable_values[4] = positions_array[4]
        self.group_variable_values[5] = positions_array[5]
        self.group_variable_values[6] = positions_array[6]
        self.group.set_joint_value_target(self.group_variable_values)
        result =  self.execute_trajectory()
        
        return result
        
    def execute_trajectory(self):
        """
        execute the motion plan.
        """
        self.plan = self.group.plan() # returns a MOTION PLAN.
        result = self.group.go(wait=True) # set the target of the group and then move the group to the specified target.
        #######
        # Fail: ABORTED: No motion plan found. No execution attempted.
        ######3
        return result

    def ee_pose(self):
        
        gripper_pose = self.group.get_current_pose()

        rospy.logdebug("EE POSE==>"+str(gripper_pose))

        return gripper_pose
        
    def ee_rpy(self, request): # rpy: raw, pitch, yaw
        
        gripper_rpy = self.group.get_current_rpy()

        return gripper_rpy