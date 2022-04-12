from openai_ros import robot_gazebo_env
import rospy
from sensor_msgs.msg import JointState
import moveit_commander
import sys
import geometry_msgs.msg
import numpy as np
import time

class PandaEnv(robot_gazebo_env.RobotGazeboEnv):
    def __init__(self):
        """
        initializes a new Panda environment. This class contain all the ROS functionalities that 
        your robot will need in order to be controlled. 
        self.controllers_list: a list of controllers.
        self.robot_name_space: robot namespace.
        self.reset_controls: reset controllers are not when resetting.
        self.JOINT_STATES_SUBSCRIBER: name of the joint states subscriber.
        self.joint_names: name of the joints.
        self.joint_states_sub = the joint states subscriber.
        self.joints: the joints.
        self.move_reach_object: object to interact with MoveIt.
        """
        rospy.logdebug("Entered Panda Env")

        self.controllers_list = ["/position_joint_trajectory_controller"]
        self.robot_name_space = "panda"
        self.reset_controls = False
        
        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(PandaEnv, self).__init__(controllers_list  = self.controllers_list,
                                                robot_name_space = self.robot_name_space,
                                                reset_controls = self.reset_controls)
        
        # We Start all the ROS related Subscribers and publishers
        self.JOINT_STATES_SUBSCRIBER = '/joint_states'
        self.joint_names = ["panda_joint1",
                          "panda_joint2",
                          "panda_joint3",
                          "panda_joint4",
                          "panda_joint5",
                          "panda_joint6",
                          "panda_joint7"]
        
        #self.gazebo.unpauseSim()
        self._check_all_systems_ready()

        self.joint_states_sub = rospy.Subscriber(self.JOINT_STATES_SUBSCRIBER, JointState, self.joints_callback)
        self.joints = JointState()
        
        # Start Services
        self.move_reach_object = MoveReach()
        
        #self.gazebo.pauseSim()
        rospy.logdebug("Panda Env init DONE")


    # RobotGazeboEnv Virtual Methods
    # ----------------------------
    def _check_all_systems_ready(self):
        """
        check if all the sensors, publishers and other simulation systems are ready.
        """
        self._check_all_sensors_ready()
        rospy.logdebug("ALL SYSTEMS READY")
    # ----------------------------

    # Methods RobotEnv will need.
    # ----------------------------
    def _check_all_sensors_ready(self):
        """
        check if all the sensors are ready.
        """
        self._check_joint_states_ready()
        rospy.logdebug("ALL SENSORS READY")
    
    def _check_joint_states_ready(self):
        """
        check if all the joints are ready.
        :returns: a list of joints.
        """
        self.joints = None
        while self.joints is None and not rospy.is_shutdown():
            try:
                self.joints = rospy.wait_for_message(self.JOINT_STATES_SUBSCRIBER, JointState, timeout=1.0)
                rospy.logdebug(str(self.JOINT_STATES_SUBSCRIBER)+" READY, the joints are =>\n" + str(self.joints))
            except:
                rospy.logerr(str(self.JOINT_STATES_SUBSCRIBER)+" not ready yet, retrying....")
        return self.joints

    def joints_callback(self, data):
        """
        joint callback.
        """
        self.joints = data

    def set_trajectory_ee(self, action):
        """
        sets the pose of the end effector based on action.
        :param action: the action. [x, y, z, x, y, z,]
        :returns: if the plan succeed.
        """
        ee_target = geometry_msgs.msg.Pose()
        # prevent sending pose that's out of range.
        # action = np.clip(action, self.position_low, self.position_high)
        ee_target.position.x = action[0]
        ee_target.position.y = action[1]
        ee_target.position.z = action[2]
        ee_target.orientation.x = action[3]
        ee_target.orientation.y = action[4]
        ee_target.orientation.z = action[5]
        ee_target.orientation.w = action[6]
        
        rospy.logdebug("set_trajectory_ee...start = \n" + str(ee_target.position))
        result = self.move_reach_object.ee_traj(ee_target)
        rospy.logdebug("set_trajectory_ee...result = "+str(result))
        
        return result
        
    def set_trajectory_joints(self, joints_values):
        """
        set the desired joint values of each joint, and execute the motion plan.
        :param joints_values: an array of joint values.
        :returns: if the plan succeed.
        """
        positions_array = [None] * 7
        positions_array[0] = joints_values["panda_joint1"]
        positions_array[1] = joints_values["panda_joint2"]
        positions_array[2] = joints_values["panda_joint3"]
        positions_array[3] = joints_values["panda_joint4"]
        positions_array[4] = joints_values["panda_joint5"]
        positions_array[5] = joints_values["panda_joint6"]
        positions_array[6] = joints_values["panda_joint7"]
        # prevent sending joint value that's out of range.
        positions_array = np.clip(positions_array, [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973], [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
 
        result = self.move_reach_object.joint_traj(positions_array)
        
        return result
        
    def create_action(self, position, orientation):
        """
        create action based on the position and the orientation.
        :param position: action's position. [x, y, z]
        :param orientation: action's orientation. [x, y, z, w]
        :returns: the action.
        """
        gripper_target = np.array(position)
        gripper_rotation = np.array(orientation)
        action = np.concatenate([gripper_target, gripper_rotation])
        
        return action
    
    def create_joints_dict(self, joints_values):
        """
        create a dictionary for the joints.
        :param joints_values: an array of joints values. (in order)
        :returns: a dictionary of joints.
        'panda_joint1': 0.0,
        'panda_joint2': 0.0,
        'panda_joint3': 0.0,
        'panda_joint4': -1.57079632679,
        'panda_joint5': 0.0,
        'panda_joint6': 1.57079632679,
        'panda_joint7': 0.785398163397,
        """
        assert len(joints_values) == len(self.joint_names), "Wrong number of joints, there should be "+str(len(self.join_names))
        joints_dict = dict(zip(self.joint_names, joints_values))
        
        return joints_dict
        
    def get_ee_pose(self):
        """
        get the pose of the end effector.
        :returns: the pose of the gripper.
         geometry_msgs/PoseStamped
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
        gripper_pose = self.move_reach_object.ee_pose()
        return gripper_pose

    # ----------------------------

    # Methods the TrainingEnvironment will need to define.
    # ----------------------------
    def _set_init_pose(self):
        raise NotImplementedError()
    
    
    def _init_env_variables(self):
        raise NotImplementedError()

    def _compute_reward(self, achieved_goal, desired_goal, info):
        raise NotImplementedError()

    def _set_action(self, action):
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        raise NotImplementedError()

    #def _set_limit(self):
    #    raise NotImplementedError()
    # ----------------------------
   
        
# Class that fulfills the Reach movement with MoveIt.
# ----------------------------
class MoveReach(object):
    """
    MoveIt interactor.
    self.robot: robot commander.
    self.scene: interface of the planning scene.
    self.group: move group commander. The group name is panda_arm.
    """
    def __init__(self):
        
        rospy.logdebug("In MoveReach init...")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.logdebug("moveit_commander initialised...")
        
        rospy.logdebug("Starting Robot Commander...")
        self.robot = moveit_commander.RobotCommander()
        rospy.logdebug("Starting Robot Commander...DONE")
        
        self.scene = moveit_commander.PlanningSceneInterface()  
        rospy.logdebug("PlanningSceneInterface initialised...DONE")
        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        rospy.logdebug("MoveGroupCommander for panda_arm initialised...DONE")
        self.group.set_planner_id("RRTConnectkConfigDefault") # the planners are defined in panda_moveit_config/ompl_planning.yaml.
        rospy.logdebug("Set planner...DONE")
        self.group.set_max_acceleration_scaling_factor(0.1)
        self.group.set_max_velocity_scaling_factor(0.1)

    def ee_traj(self, pose):
        """
        set target end effector pose and execute the motion plan.
        :param pose: an array of the desired pose of the ed effector.
        :returns: if the plan succeed.
        """
        self.group.set_pose_target(pose)
        result = self.execute_trajectory()
        
        return result
        
    def joint_traj(self, joint_value):
        """
        set target joint values and execute the motion plan.
        :param joint_value: an array of joint values.
        :returns: if the plan succeed.
        """
        self.group_variable_values = self.group.get_current_joint_values() # get the correct dimension.
        self.group_variable_values[0] = joint_value[0]
        self.group_variable_values[1] = joint_value[1]
        self.group_variable_values[2] = joint_value[2]
        self.group_variable_values[3] = joint_value[3]
        self.group_variable_values[4] = joint_value[4]
        self.group_variable_values[5] = joint_value[5]
        self.group_variable_values[6] = joint_value[6]
        self.group.set_joint_value_target(self.group_variable_values)
        result =  self.execute_trajectory()
        
        return result
        
    def execute_trajectory(self):
        """
        execute the motion plan.
        :returns: if the plan succeed.
        """
        # self.plan = self.group.plan() # returns a MOTION PLAN. go already plans for us.
        # slow if wait == True, but it is probably necessary to set it as True?
        result = self.group.go(wait = True) # set the target of the group and then move the group to the specified target.
        return result

    def ee_pose(self):
        """
        get the pose of the end effector.
        :returns: the pose of the gripper.
        """
        gripper_pose = self.group.get_current_pose()

        rospy.logdebug("EE POSE==>"+str(gripper_pose))

        return gripper_pose