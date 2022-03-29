import rospy
import gym
from gym.utils import seeding
from .gazebo_connection import GazeboConnection
from .controllers_connection import ControllersConnection
#https://bitbucket.org/theconstructcore/theconstruct_msgs/src/master/msg/RLExperimentInfo.msg
from openai_ros.msg import RLExperimentInfo

# https://github.com/openai/gym/blob/master/gym/core.py
class RobotGazeboEnv(gym.Env):

    def __init__(self, robot_name_space, controllers_list, reset_controls, start_init_physics_parameters=True, reset_world_or_sim = "NO_RESET_SIM"):#reset_world_or_sim: "SIMULATION" "NO_RESET_SIM"
        """
        The superclass of all. Includes the definitions of the highest level.
        self.gazebo: connection of gazebo.
        self.controllers_object: connection to controllers.
        self.reset_controls: reset controls or not.
        self.episode_num: current episode number.
        self.cumulated_episode_reward: the episode's current cumulated reward.
        self.reward_pub: reward's publisher.
        """
        # To reset Simulations
        rospy.logdebug("START init RobotGazeboEnv")
        self.gazebo = GazeboConnection(start_init_physics_parameters,reset_world_or_sim)
        self.controllers_object = ControllersConnection(namespace = robot_name_space, controllers_list = controllers_list) ### set controller and namespace here.
        self.reset_controls = reset_controls
        self.seed()

        self.episode_num = 0
        self.cumulated_episode_reward = 0
        self.reward_pub = rospy.Publisher('/openai/reward', RLExperimentInfo, queue_size = 1)
        rospy.logdebug("END init RobotGazeboEnv")

    def seed(self, seed = None):
        """
        set a function that generate random values.
        :param seed: the seed.
        :returns: [the seed].
        """
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        """
        execute an action in a time step, and get related data.
        :param action: the action.
        :returns: obs, reward, done, info.
        """
        rospy.logdebug("START STEP")

        self.gazebo.unpauseSim()
        self._set_action(action) #### This will change the simulator's pose.
        obs = self._get_obs()
        self.gazebo.pauseSim() # pause after getting obs so that we can get its states.
        done = self._is_done(obs)
        info = {}
        reward = self._compute_reward(obs, done)
        self.cumulated_episode_reward += reward
        rospy.logdebug("END STEP")

        return obs, reward, done, info

    def reset(self):
        """
        reset the environment. have to call this before step.
        :returns: the observation after reset.
        """
        rospy.logdebug("resetting RobotGazeboEnvironment")
        self._reset_sim()
        self._init_env_variables()
        self._update_episode()
        obs = self._get_obs()
        rospy.logdebug("resetting RobotGazeboEnvironment DONE")
        return obs

    def close(self):
        """
        close the environment.
        """
        rospy.logdebug("closing RobotGazeboEnvironment")
        rospy.signal_shutdown("closing RobotGazeboEnvironment")

    # Extension methods
    # ----------------------------
    def _update_episode(self): # for internal use.
        """
        publishes the cumulated reward of the episode and increases the episode number by one.
        """
        rospy.logdebug("PUBLISHING REWARD...")
        self._publish_reward_topic(
                                    self.cumulated_episode_reward,
                                    self.episode_num
                                    )
        rospy.logdebug("PUBLISHING REWARD...DONE = "+str(self.cumulated_episode_reward)+",EP = "+str(self.episode_num))
        self.episode_num += 1
        self.cumulated_episode_reward = 0

    def _publish_reward_topic(self, reward, episode_number = 1):
        """
        publish the reward to the reward topic.
        :param reward: reward of the episode.
        :param episode_number: the number of the episode.
        """
        reward_msg = RLExperimentInfo()
        reward_msg.episode_number = episode_number
        reward_msg.episode_reward = reward
        self.reward_pub.publish(reward_msg)

    def _reset_sim(self):
        """
        resets the simulation.
        :returns: if simulation successfully reset.
        """
        rospy.logdebug("RESET SIM START")
        if self.reset_controls :
            rospy.logdebug("RESET CONTROLLERS")
            # reset controllers, check all systems ready and set robot to init pose.
            self.gazebo.unpauseSim()
            self.controllers_object.reset_controllers()
            self._check_all_systems_ready()
            self._set_init_pose()
            self.gazebo.pauseSim()
            self.gazebo.resetSim()
            self.gazebo.unpauseSim()
            self.controllers_object.reset_controllers()
            self._check_all_systems_ready()
            self.gazebo.pauseSim()
        else:
            rospy.logdebug("DONT RESET CONTROLLERS")
            # check all systems ready and set robot to init pose.
            self.gazebo.unpauseSim()
            self._check_all_systems_ready()
            self._set_init_pose()
            self.gazebo.pauseSim()
            self.gazebo.resetSim()
            self.gazebo.unpauseSim()
            self._check_all_systems_ready()
            self.gazebo.pauseSim()

        rospy.logdebug("RESET SIM END")
        return True


# Abstract Methods.
# ----------------------------
    def _set_init_pose(self): # TrainEnv.
        raise NotImplementedError()

    def _check_all_systems_ready(self): # RobotEnv.
        raise NotImplementedError()

    def _get_obs(self): # TrainEnv.
        raise NotImplementedError()

    def _init_env_variables(self): # TrainEnv.
        raise NotImplementedError()

    def _set_action(self, action): # TrainEnv.
        raise NotImplementedError()

    def _is_done(self, observations): # TrainEnv.
        raise NotImplementedError()

    def _compute_reward(self, observations, done): # TrainEnv.
        raise NotImplementedError()

    def _env_setup(self, initial_qpos): # TrainEnv.
        raise NotImplementedError()
    
    #def _set_limit(self):
    #    raise NotImplementedError()

