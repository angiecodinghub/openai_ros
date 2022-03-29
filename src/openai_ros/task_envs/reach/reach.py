from gym import spaces, utils
from gym.envs.registration import register
import rospy
from openai_ros.robot_envs import panda_env
import numpy as np
import time

timestep_limit_per_episode = 1000

# register environment in gym.
register(
        id = 'PandaReach-v2',
        entry_point = 'openai_ros.task_envs.reach.reach:ReachEnv',
        max_episode_steps = timestep_limit_per_episode
    )

class ReachEnv(panda_env.PandaEnv, utils.EzPickle):
    """
    This class provides all the context for the reach task we want Panda to learn. 
    self.control_type: what kind of control to learn. "ee" or "joint". affect self.n_actions.
    self.n_actions: number of actions.
    self.has_object: if there's object in the environment.
    self.block_gripper: if we want to ignore gripper.
    self.distance_threshold: threshold distance to the goal.
    self.reward_type: the type of reward.
    self.init_pos: initial pose.
    self.n_substeps: number of substeps.
    self.gripper_extra_height: height of the gripper.
    self.target_in_the_air: if the target is in the air.
    self.target_offset: offset to the target.
    self.obj_range: the range of the object.
    self.target_range: the max distance from start to target.
    self.action_space: the action space.
    self.observation_space: the observation space.
    self.initial_gripper_pos: initial gripper position.
    """
    def __init__(self, control_type="ee"): ## default control_type is ee.
        rospy.logdebug("entered Reach Env")
        # set params.
        self.control_type = control_type
        self.get_params()

        panda_env.PandaEnv.__init__(self)
        utils.EzPickle.__init__(self)

        rospy.logdebug("reach env - setup")
        self._env_setup(self.init_pos)
 
        rospy.logdebug("reach env - check obs")
        obs = self._get_obs()
        rospy.logdebug("current obs:\n")
        rospy.logdebug(obs)

        self.action_space = spaces.Box(-1., 1., shape=(self.n_actions,), dtype='float32')
        self.observation_space = spaces.Dict(
            dict(
                observation=spaces.Box(-10., 10., shape=(6,), dtype=np.float32),
                desired_goal=spaces.Box(-10., 10., shape=obs['achieved_goal'].shape, dtype=np.float32),
                achieved_goal=spaces.Box(-10., 10., shape=obs['desired_goal'].shape, dtype=np.float32),
            )
        )
        #self._set_limit()
        rospy.logdebug("Reach Env init DONE")


    def get_params(self):
        """
        get configuration parameters.
        """
        if self.control_type == "ee":
            self.n_actions = 3 # ee position
        else:
            self.n_actions = 7 # joint angle.
        self.has_object = False
        self.block_gripper = True
        self.distance_threshold = 0.05
        self.reward_type = "sparse"
        self.init_pos = { # 90 degree bend in the elbow. from OLD panda.launch in franka_gazebo.
            'panda_joint1': 0.0,
            'panda_joint2': 0.0,
            'panda_joint3': 0.0,
            'panda_joint4': -1.57079632679,
            'panda_joint5': 0.0,
            'panda_joint6': 1.57079632679,
            'panda_joint7': 0.785398163397,
        }
        self.n_substeps = 20
        self.gripper_extra_height = 0.2
        self.target_in_the_air = True
        self.target_offset = 0.0
        self.obj_range = 0.15
        self.target_range = 0.15

    def robot_get_obs(self, joints):
        """
        returns current joint positions and joint velocities of Panda.
        :param joints: a list of joints.
        :returns: current joint positions and joint velocities.
        """
        if joints.position is not None and joints.name:
            names = [n for n in joints.name]
            i = 0
            r = 0
            # count the number of joints.
            for name in names:
                r += 1
                
            return (
                np.array([joints.position[i] for i in range(r)]),
                np.array([joints.velocity[i] for i in range(r)]),
            )
        return np.zeros(0), np.zeros(0)


    # RobotGazeboEnv's virtual methods.
    # --------------------------------
    def _set_init_pose(self):
        """
        sets Panda in its init pose.
        :returns: if the plan succeed.
        """
        self.gazebo.unpauseSim()
        result = self.set_trajectory_joints(self.init_pos)

        return result

    def _init_env_variables(self):
        """
        init environment variables.
        """
        self.n_actions = 3
        self.has_object = False
        self.block_gripper = True
        self.distance_threshold = 0.05
        self.reward_type = "sparse"
        if self.control_type == "ee":
            self.n_actions = 3 # ee position
        else:
            self.n_actions = 7 # joint angle.
        self.init_pos = { # 90 degree bend in the elbow. from OLD panda.launch in franka_gazebo.
            'panda_joint1': 0.0,
            'panda_joint2': 0.0,
            'panda_joint3': 0.0,
            'panda_joint4': -1.57079632679,
            'panda_joint5': 0.0,
            'panda_joint6': 1.57079632679,
            'panda_joint7': 0.785398163397,
        }
        self.n_substeps = 20
        self.gripper_extra_height = 0.2
        self.target_in_the_air = True
        self.target_offset = 0.0
        self.obj_range = 0.15
        self.target_range = 0.15

    def _set_action(self, action):
        """
        move the robot based on the action variable given.
        reference: ee_displacement_to_target_arm_angles in panda.py.
        :param action: the action.
        """
        if self.control_type == "ee":
            assert action.shape == (3,)
            action = action.copy()  # ensure action don't change
            action = np.clip(action, self.action_space.low, self.action_space.high)
            ee_displacement = action[:3] * 0.05
            current_obs = self._get_obs()
            next_obs = current_obs['observation'][:3] + ee_displacement
            next_obs[2] = max(0, next_obs[2])
            rot_ctrl = [1., 0., 0., 0.] ### PLACEHOLDER FOR ORIENTATION OF EE.
            action = np.concatenate([next_obs, rot_ctrl])
            self.set_trajectory_ee(action)  
        else: # "joint"
            assert action.shape == (7,)
            action = action.copy()  # ensure action don't change
            action = np.clip(action, self.action_space.low, self.action_space.high)
            joint_displacement = action * 0.05
            current_joint, _ = self.robot_get_obs(self.joints)
            next_joint = current_joint[:7] + joint_displacement
            # make joint 
            joint_degree = {
            'panda_joint1': float(next_joint[0]),
            'panda_joint2': float(next_joint[1]),
            'panda_joint3': float(next_joint[2]),
            'panda_joint4': float(next_joint[3]),
            'panda_joint5': float(next_joint[4]),
            'panda_joint6': float(next_joint[5]),
            'panda_joint7': float(next_joint[6]),
            }
            self.set_trajectory_joints(joint_degree)
    
     
    def _get_obs(self):
        """
        get the observation of the state now.
        :returns: observation (ee position and velocity), achieved goal and desired goal.
        """
        grip_pos = self.get_ee_pose()
        grip_pos_array = np.array([grip_pos.pose.position.x, grip_pos.pose.position.y, grip_pos.pose.position.z])
        grip_vel_array = np.array([0., 0., 0.])
        achieved_goal = self._sample_achieved_goal(grip_pos_array)
        obs = np.concatenate([
            grip_pos_array, grip_vel_array,
        ])

        return {
            'observation': obs.copy(),
            'achieved_goal': achieved_goal.copy(),
            'desired_goal': self.goal.copy(),
        }

    def _is_done(self, observations):
        """
        check if the episode and is done.
        :param observations: the observations of the state.
        :returns: if the episode is done.
        """
        if np.linalg.norm(observations['achieved_goal'] - observations['desired_goal'], axis = -1) < self.distance_threshold:
            return True
        else:
            return False

    def _compute_reward(self, observations, done):
        """
        return the reward based on the observations given.
        :param observations: the observations of the state.
        :param done: if the episode is done.
        :returns: the reward.
        """
        if done:
            return -0
        distance = np.linalg.norm(observations['achieved_goal'] - observations['desired_goal'], axis = -1)
        if self.reward_type == "sparse":
            return -np.array(distance > self.distance_threshold, dtype = np.float64)
        else:
            return -distance
    
    #def _set_limit(self):
    #    self.position_low = self.initial_gripper_pos + [-self.distance_threshold] * len(self.initial_gripper_pos)
    #    self.position_high = self.initial_gripper_pos + [self.distance_threshold] * len(self.initial_gripper_pos)
    #    self.joint_low = []
    #    self.joint_high = []
    # --------------------------------

    # internal methods for ReachEnv.
    # -----------------------------
    def _sample_achieved_goal(self, grip_pos_array):
        """
        return the achieved goal.
        :param grip_pos_array: the position array of the gripper.
        :returns: the achieved goal.
        """
        achieved_goal = grip_pos_array.copy()
        return achieved_goal

    def _sample_goal(self):
        """
        sample a goal according to self.target_range.
        :returns: the goal.
        """
        goal = self.initial_gripper_pos[:3] + self.np_random.uniform(-self.target_range, self.target_range, size = 3)
        return goal

    def _env_setup(self, initial_pos):
        """
        setup the init pose of the robot and the gripper, and set a goal.
        :param initial_pos: initial pose of the joints.
        """
        self.gazebo.unpauseSim()
        self.set_trajectory_joints(initial_pos)

        # get the position of the end effector.
        gripper_pos = self.get_ee_pose()
        gripper_pose_array = np.array([gripper_pos.pose.position.x, gripper_pos.pose.position.y, gripper_pos.pose.position.z])
        self.initial_gripper_pos = gripper_pose_array.copy()
            
        self.goal = self._sample_goal()
    # -----------------------------