from gym import spaces, utils
from gym.envs.registration import register
import rospy
from openai_ros.robot_envs import panda_env
import numpy as np
import time

timestep_limit_per_episode = 1000 # Can be any Value
# register my own environment in gym.
register(
        id='PandaReach-v2',
        entry_point='openai_ros.task_envs.reach.reach:ReachEnv',
        max_episode_steps=timestep_limit_per_episode
    )

class ReachEnv(panda_env.PandaEnv, utils.EzPickle):
    """
    This class provides all the context for the reach task we want Panda to learn. 
    It depends on both the task and on the robot.
    """
    def __init__(self):
        rospy.logdebug("entered Reach Env")
        # Only variable needed to be set here
        
        self.get_params()

        panda_env.PandaEnv.__init__(self)
        utils.EzPickle.__init__(self)

        print("reach env - setup")
        self._env_setup(initial_qpos = self.init_pos)
 
        print ("reach env - check obs")
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
        rospy.logdebug("Reach Env settings DONE")


    def get_params(self):
        """
        Get configuration parameters.
        """
        self.n_actions = 3
        self.has_object = False
        self.block_gripper = True
        self.distance_threshold = 0.05
        self.reward_type = "sparse"
        self.control_type = "ee" # we only control where the ee is at.
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

    def robot_get_obs(self, data):
        """
        Returns all joint positions and velocities associated with a robot.
        """
        if data.position is not None and data.name:
            #names = [n for n in data.name if n.startswith('robot')]
            names = [n for n in data.name]
            i = 0
            r = 0
            for name in names:
                r += 1
                
            return (
                np.array([data.position[i] for i in range(r)]),
                np.array([data.velocity[i] for i in range(r)]),
            )
        return np.zeros(0), np.zeros(0)


    # RobotGazeboEnv's virtual methods.
    # --------------------------------
    def _set_init_pose(self):
        """
        Sets the Robot in its init pose
        """
        self.gazebo.unpauseSim()
        self.set_trajectory_joints(self.init_pos)

        return True

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        self.n_actions = 3
        self.has_object = False
        self.block_gripper = True
        self.distance_threshold = 0.05
        self.reward_type = "sparse"
        self.control_type = "ee" # we only control where the ee is at.
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
        Move the robot based on the action variable given.
        ref: ee_displacement_to_target_arm_angles in panda.py.
        """
        assert action.shape == (3,)
        action = action.copy()  # ensure action don't change
        action = np.clip(action, self.action_space.low, self.action_space.high)
        ee_displacement = action[:3] * 0.05
        current_obs = self._get_obs()
        print("current observation:", current_obs['observation'][:3])
        print("raw ee displacement:", action[:3])
        print("real ee displacement:", ee_displacement)
        next_obs = current_obs['observation'][:3] + ee_displacement
        next_obs[2] = max(0, next_obs[2])
        print("next observation:", next_obs)
        # print("desired observation:", current_obs['desired_goal'][:3])
        rot_ctrl = [1., 0., 0., 0.] ### PLACEHOLDER FOR ORIENTATION OF EE.
        action = np.concatenate([next_obs, rot_ctrl])
        self.set_trajectory_ee(action)
        time.sleep(1) ##### WAIT FOR 1s      
        #
        """
        rot_ctrl = [1., 0., 1., 0.] ### PLACEHOLDER FOR ORIENTATION OF QUARTERNIONS.
        action = np.concatenate([ee_displacement, rot_ctrl])
        #for i in range(self.n_substeps): ## do the action substeps time.
        self.set_trajectory_ee(action)
        time.sleep(2) ##### WAIT FOR 2s          
        """
 
    
     
    def _get_obs(self):
        """
        Here we define what sensor data do we want our robot to know.
        """
        # TODO
        grip_pos = self.get_ee_pose() # pose
        grip_pos_array = np.array([grip_pos.pose.position.x, grip_pos.pose.position.y, grip_pos.pose.position.z])
        grip_vel_array = np.array([0., 0., 0.])
        #robot_qpos, robot_qvel = self.robot_get_obs(self.joints) 
        # gripper's velocity
        ##### EE VELOCITY IS NOT ACCESSIBLE IN MOVEIT ########
        #gripper_state = robot_qpos[-1:] # -2, not a good fix tho.
        #gripper_vel = robot_qvel[-2:] #* dt  # change to a scalar if the gripper is made symmetric
        #print("got ee pose array gggg", gripper_vel) 
        achieved_goal = self._sample_achieved_goal(grip_pos_array)
        obs = np.concatenate([
        #    grip_pos_array, gripper_state, gripper_vel,
            grip_pos_array, grip_vel_array,
        ])

        return {
            'observation': obs.copy(),
            'achieved_goal': achieved_goal.copy(),
            'desired_goal': self.goal.copy(),
        }

    def _is_done(self, observations):
        """
        Decide if episode is done based on the observations
        """
        if np.linalg.norm(observations['achieved_goal'] - observations['desired_goal'], axis = -1) < self.distance_threshold:
            return True
        else:
            return False

    def _compute_reward(self, observations, done):
        """
        Return the reward based on the observations given
        """
        # TODO
        if done:
            return -0
        distance = np.linalg.norm(observations['achieved_goal'] - observations['desired_goal'], axis = -1)
        if self.reward_type == "sparse":
            return -np.array(distance > self.distance_threshold, dtype=np.float64)
        else:
            return -distance
    # --------------------------------

    # internal methods for ReachEnv.
    # -----------------------------
    def _sample_achieved_goal(self, grip_pos_array):
        achieved_goal = grip_pos_array.copy()
        return achieved_goal

    def _sample_goal(self):
        goal = self.initial_gripper_xpos[:3] + self.np_random.uniform(-0.15, 0.15, size=3)
        return goal

    def _env_setup(self, initial_qpos):
        print("Desired Init Pos:", initial_qpos)
        self.gazebo.unpauseSim()
        start = time.time()
        self.set_trajectory_joints(initial_qpos) ###### THIS SHOULD SET THE ARM IN THE DESIRED POSITION, SUCCEED.
        end = time.time()
        print("env setup execution time:", end - start)

        # Move gripper into position.
        # gripper_target = np.array([0.498, 0.005, 0.431 + self.gripper_extra_height])# + self.sim.data.get_site_xpos('robot0:grip')
        # gripper_rotation = np.array([1., 0., 1., 0.])
        # action = np.concatenate([gripper_target, gripper_rotation])
        # self.set_trajectory_ee(action)

        # Extract information for sampling goals.
        gripper_pos = self.get_ee_pose()
        gripper_pose_array = np.array([gripper_pos.pose.position.x, gripper_pos.pose.position.y, gripper_pos.pose.position.z])
        self.initial_gripper_xpos = gripper_pose_array.copy()
            
        self.goal = self._sample_goal()
        self._get_obs()
    
    # -----------------------------