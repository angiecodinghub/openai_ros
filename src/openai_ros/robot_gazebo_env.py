import rospy
import gym
from gym.utils import seeding
from .gazebo_connection import GazeboConnection
from .controllers_connection import ControllersConnection
#https://bitbucket.org/theconstructcore/theconstruct_msgs/src/master/msg/RLExperimentInfo.msg
from openai_ros.msg import RLExperimentInfo
import time

# https://github.com/openai/gym/blob/master/gym/core.py
class RobotGazeboEnv(gym.Env):

    def __init__(self, robot_name_space, controllers_list, reset_controls, start_init_physics_parameters=True, reset_world_or_sim="SIMULATION"):

        # To reset Simulations
        rospy.logdebug("START init RobotGazeboEnv")
        self.gazebo = GazeboConnection(start_init_physics_parameters,reset_world_or_sim)
        self.controllers_object = ControllersConnection(namespace = robot_name_space, controllers_list = controllers_list) ### set controller and namespace here.
        self.reset_controls = reset_controls
        self.seed()

        # Set up ROS related variables
        self.episode_num = 0
        self.cumulated_episode_reward = 0
        self.reward_pub = rospy.Publisher('/openai/reward', RLExperimentInfo, queue_size=1)
        rospy.logdebug("END init RobotGazeboEnv")

    # Env methods
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        """
        Function executed each time step.
        We ainput an action, execute it in a time step, and retrieve the
        observations generated by that action.
        :param action:
        :return: obs, reward, done, info
        """

        """
        Here we should convert the action num to movement action, execute the action in the
        simulation and get the observations result of performing that action.
        """
        rospy.logdebug("START STEP OpenAI ROS")

        self.gazebo.unpauseSim()
        start = time.time()
        self._set_action(action) #### This will change the simulator's pose.
        done = time.time()
        print("step set action execution time:", done - start)
        self.gazebo.pauseSim()
        obs = self._get_obs()
        done = self._is_done(obs)
        info = {}
        reward = self._compute_reward(obs, done)
        self.cumulated_episode_reward += reward

        rospy.logdebug("END STEP OpenAI ROS")

        return obs, reward, done, info

    def reset(self):
        """
        reset the environment. Have to call before step.
        """
        rospy.logdebug("Reseting RobotGazeboEnvironment")
        print("resetting")
        self._reset_sim()
        self._init_env_variables()
        self._update_episode()
        obs = self._get_obs()
        print("done resetting")
        rospy.logdebug("END Reseting RobotGazeboEnvironment")
        return obs

    def close(self):
        """
        Function executed when closing the environment.
        Use it for closing GUIS and other systems that need closing.
        :return:
        """
        rospy.logdebug("Closing RobotGazeboEnvironment")
        rospy.signal_shutdown("Closing RobotGazeboEnvironment")

    def _update_episode(self): # for internal use.
        """
        Publishes the cumulated reward of the episode and
        increases the episode number by one.
        :return:
        """
        rospy.logdebug("PUBLISHING REWARD...")
        self._publish_reward_topic(
                                    self.cumulated_episode_reward,
                                    self.episode_num
                                    )
        rospy.logdebug("PUBLISHING REWARD...DONE="+str(self.cumulated_episode_reward)+",EP="+str(self.episode_num))

        self.episode_num += 1
        self.cumulated_episode_reward = 0


    def _publish_reward_topic(self, reward, episode_number=1):
        """
        This function publishes the given reward in the reward topic for
        easy access from ROS infrastructure.
        :param reward:
        :param episode_number:
        :return:
        """
        reward_msg = RLExperimentInfo()
        reward_msg.episode_number = episode_number
        reward_msg.episode_reward = reward
        self.reward_pub.publish(reward_msg)

    # Extension methods
    # ----------------------------

    def _reset_sim(self):
        """
        Resets a simulation.
        """
        rospy.logdebug("RESET SIM START")
        if self.reset_controls :
            rospy.logdebug("RESET CONTROLLERS")
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

    def _set_init_pose(self): # TrainEnv.
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()

    def _check_all_systems_ready(self): # RobotEnv.
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        raise NotImplementedError()

    def _get_obs(self): # TrainEnv.
        """Returns the observation.
        """
        raise NotImplementedError()

    def _init_env_variables(self): # TrainEnv.
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _set_action(self, action): # TrainEnv.
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _is_done(self, observations): # TrainEnv.
        """Indicates whether or not the episode is done ( the robot has fallen for example).
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done): # TrainEnv.
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _env_setup(self, initial_qpos):
        """Initial configuration of the environment. Can be used to configure initial state
        and extract information from the simulation.
        """
        raise NotImplementedError()

