"""
bridge_env.py

Py Bridge Designer
by Nathan Hartzler
"""
import gymnasium as gym
import numpy as np
from gymnasium import spaces
from typing import Tuple
from py_bridge_designer.bridge import Bridge, BridgeError


class BridgeEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 30}

    def __init__(self, render_mode=None, load_scenario_index=None, test_print=False):
        self.reward_range = (-np.inf, 0)
        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode
        self.test_print = test_print

        # Select a random load_scenario_index if needed
        if load_scenario_index is None:
            self.load_scenario_index = self._rand_load_scenario_index()
        else:
            self.load_scenario_index = load_scenario_index

        # Init the bridge
        self.bridge = Bridge(self.load_scenario_index)

        # Define action space
        action_size = self.bridge.get_size_of_add_member_parameters()

        self.action_space = spaces.Box(
            low=min(action_size),
            high=max(action_size),
            shape=(len(action_size),),
            dtype=np.int16)

        # Define observation space
        self.observation_space = spaces.Box(
            low=0,
            high=1,
            shape=self._get_observation().shape,
            dtype=np.int8)

    def _rand_load_scenario_index(self) -> int:
        # return int(self.np_random.uniform(low=0, high=392))
        # trying without seeded rng
        return int(np.random.uniform(low=0, high=392))

    @staticmethod
    def _calculate_reward(bridge_valid: bool,
                          bridge_error: BridgeError,
                          bridge_cost: int) -> Tuple[float, bool]:

        _reward = round(-(bridge_cost * .000001), 4)
        if bridge_error == BridgeError.BridgeNoError:
            if bridge_valid:
                complete = True
                return _reward, complete
            else:
                complete = False
                return 1, complete
        elif bridge_error == BridgeError.BridgeAtMaxJoints:
            complete = True
            penalty = 10
            return _reward * penalty, complete
        elif bridge_error == BridgeError.BridgeJointOutOfBounds or bridge_error == BridgeError.BridgeJointsAreEqual:
            complete = False
            return -40, complete
        elif bridge_error == BridgeError.BridgeJointNotConnected:
            complete = False
            return -2, complete
        else:
            complete = True
            print("Error! Unknown BridgeError type in _calculate_reward")
            penalty = 10
            return -bridge_cost * penalty, complete

    def _get_observation(self):
        """This should not be called before reset()"""
        return np.array(self.bridge.get_state(), dtype=np.int8)

    def _get_info(self, current_error=BridgeError.BridgeNoError, bridge_valid=False):
        """This should not be called before reset()"""
        return {
            "scenario_id": self.bridge.load_scenario.desc.id,
            "scenario_site_cost": self.bridge.load_scenario.desc.site_cost,
            "current_error": current_error,
            "bridge_valid": bridge_valid
        }

    def reset(self, seed=None, load_scenario_index=None, options=None):
        super().reset(seed=seed)

        # Select a random load_scenario_index if needed
        if load_scenario_index is None:
            self.load_scenario_index = self._rand_load_scenario_index()
        else:
            self.load_scenario_index = load_scenario_index

        # Init the bridge
        self.bridge = Bridge(self.load_scenario_index)

        # Define action space
        action_size = self.bridge.get_size_of_add_member_parameters()

        self.action_space = spaces.Box(
            low=0,
            high=max(action_size),
            shape=(len(action_size),),
            dtype=np.int16,
            seed=seed)

        # Define observation space
        self.observation_space = spaces.Box(
            low=0,
            high=1,
            shape=self._get_observation().shape,
            dtype=np.int8,
            seed=seed)

        # Return the observation and info
        observation = self._get_observation()
        info = self._get_info()

        return observation, info

    def _clip_action(self, action):
        sizes = self.bridge.get_size_of_add_member_parameters()
        new_action = []
        for i in range(len(sizes)):
            max_x = sizes[i]
            x = action[i]
            new_action.append(x if x < max_x else max_x - 1)
        return new_action

    def step(self, action):
        action = self._clip_action(action)
        bridge_error = self.bridge.add_member(
            action[0], action[1], action[2], action[3], action[4], action[5], action[6])

        bridge_valid, bridge_cost = self.bridge.analyze(self.test_print)
        reward, terminated = self._calculate_reward(
            bridge_valid, bridge_error, bridge_cost)

        observation = self._get_observation()
        info = self._get_info(current_error=bridge_error,
                              bridge_valid=bridge_valid)

        return observation, reward, terminated, False, info

    def render(self):
        if self.render_mode == "rgb_array":
            return self.bridge.get_image()
