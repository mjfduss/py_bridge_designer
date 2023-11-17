import statistics
import gymnasium as gym
import numpy as np
from gymnasium import spaces
from typing import Tuple
from bridge import Bridge, BridgeError
from stable_baselines3.common.env_checker import check_env


class BridgeEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 4}

    def __init__(self, render_mode=None, options={"load_scenario_index": None, "test_print": False}):
        self.reward_range = (-np.inf, 0)
        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode

        # Select a random load_scenario_index if needed
        if options is not None and options["load_scenario_index"] is None:
            options["load_scenario_index"] = int(
                np.random.uniform(low=0, high=392))
        else:
            options = {
                "load_scenario_index": int(
                    np.random.uniform(low=0, high=392)),
                "test_print": False
            }
        self.options = options

        # Init the bridge
        self.bridge = Bridge(self.options["load_scenario_index"])

        # Define action space
        action_size = self.bridge.get_size_of_add_member_parameters()

        self.action_space = spaces.MultiDiscrete(
            nvec=action_size, dtype=np.int16)

        # Define observation space
        self.observation_space = spaces.MultiBinary(
            n=[2, self.bridge.matrix_y, self.bridge.matrix_x])

    def _rand_load_scenario_index(self) -> int:
        return int(self.np_random.uniform(low=0, high=392))

    def _calculate_reward(self,
                          bridge_valid: bool,
                          bridge_error: BridgeError,
                          bridge_cost: float) -> Tuple[int, bool]:
        if bridge_valid:
            terminated = True
            return -bridge_cost, terminated
        else:
            if bridge_error == BridgeError.BridgeAtMaxJoints:
                terminated = True
                penalty = 100
                return -bridge_cost * penalty, terminated
            elif bridge_error == BridgeError.BridgeJointOutOfBounds:
                terminated = False
                return -2, terminated
            else:
                terminated = False
                return -1, terminated

    def _get_observation(self):
        """This should not be called before reset()"""
        return np.array(self.bridge.get_state(), dtype=np.int8)

    def _get_info(self, current_error=BridgeError.BridgeNoError):
        """This should not be called before reset()"""
        return {
            "scenario_id": self.bridge.load_scenario.desc.id,
            "scenario_site_cost": self.bridge.load_scenario.desc.site_cost,
            "current_error": current_error
        }

    def reset(self, seed=None, options={"load_scenario_index": None, "test_print": False}):
        super().reset(seed=seed)

        # Select a random load_scenario_index if needed
        if options is not None and options["load_scenario_index"] is None:
            options["load_scenario_index"] = self._rand_load_scenario_index()
        else:
            options = {
                "load_scenario_index": self._rand_load_scenario_index(),
                "test_print": False
            }
        self.options = options

        # Init the bridge
        self.bridge = Bridge(self.options["load_scenario_index"])

        # Define action space
        action_size = self.bridge.get_size_of_add_member_parameters()

        self.action_space = spaces.MultiDiscrete(
            nvec=action_size, dtype=np.int16, seed=seed)

        # Define observation space
        self.observation_space = spaces.MultiBinary(
            n=[2, self.bridge.matrix_y, self.bridge.matrix_x],
            seed=seed)

        # Return the observation and info
        observation = self._get_observation()
        info = self._get_info()

        return observation, info

    def step(self, action):
        bridge_error = self.bridge.add_member(
            action[0], action[1], action[2], action[3], action[4], action[5], action[6])

        bridge_valid, bridge_cost = self.bridge.analyze(
            self.options["test_print"])

        reward, terminated = self._calculate_reward(
            bridge_valid, bridge_error, bridge_cost)

        observation = self._get_observation()
        info = self._get_info(current_error=bridge_error)

        return observation, reward, terminated, False, info


# Testing code
env = BridgeEnv()
check_env(env)
"""
EPISODES = 10
step_counts = []
for e in range(EPISODES):
    obs = env.reset()
    done = False
    step_count = 0
    rewards = []
    terminal_reward = 0
    terminal_error = 0
    print("=====================================")
    print(f"Episode {e+1}")
    print("=====================================")
    while not done:
        action = env.action_space.sample()  # would pass obs to real network
        obs, reward, terminated, _, info = env.step(action)
        rewards.append(reward)
        if step_count % 10 == 0:
            print(f"Step: {step_count}; Action: {action}")
            print(f"Reward: {reward}; Error: {info['current_error']}")
        if terminated:
            terminal_reward = reward
            terminal_error = info['current_error']
            done = True
        else:
            step_count += 1
    print(f"~~~~~~~~ Episode {e+1} done ~~~~~~~~")
    print(f"~~~~~~~~ Step Total: {step_count+1}")
    print(f"~~~~~~~~ Mean Step Rewards: {statistics.mean(rewards[:-1])}")
    print(f"~~~~~~~~ Terminal Reward: {terminal_reward}")
    print(f"~~~~~~~~ Total Rewards: {sum(rewards)}")
    print(f"~~~~~~~~ Terminal Error: {terminal_error}")
    step_counts.append(step_count)
print(f"Mean Steps: {statistics.mean(step_counts)}")
"""
"""
# load_scenario_index=6
# lower deck joints     [(0, 0), (16, 0), (32, 0), (48, 0), (64, 0), (80, 0)])
# guess at upper joints [(8, 16), (24, 16), (40, 16), (56, 16), (72, 16)]
# material = 0, section = 0, size = 18
valid_actions = [
    [0, 0, 8, 16, 0, 0, 18],
    [0, 0, 16, 0, 0, 0, 18],
    [16, 0, 8, 16, 0, 0, 18],
    [16, 0, 24, 16, 0, 0, 18],
    [32, 0, 24, 16, 0, 0, 18],
    [16, 0, 32, 0, 0, 0, 18],
    [32, 0, 48, 0, 0, 0, 18],
    [32, 0, 40, 16, 0, 0, 18],
    [48, 0, 40, 16, 0, 0, 18],
    [48, 0, 64, 0, 0, 0, 18],
    [48, 0, 56, 16, 0, 0, 18],
    [64, 0, 56, 16, 0, 0, 18],
    [64, 0, 80, 0, 0, 0, 18],
    [64, 0, 72, 16, 0, 0, 18],
    [80, 0, 72, 16, 0, 0, 18],
    [56, 16, 72, 16, 0, 0, 18],
    [56, 16, 40, 16, 0, 0, 18],
    [24, 16, 40, 16, 0, 0, 18],
    [24, 16, 8, 16, 0, 0, 18]
]

for a in valid_actions:
    # a = env.action_space.sample()
    member_added = env.bridge.add_member(
        a[0], a[1], a[2], a[3], a[4], a[5], a[6]
    )
    if not member_added:
        print("member not added from action:", a)

valid, cost = env.bridge.analyze(test_print=False)
print("bridge valid:", valid)
print(f"bridge cost {cost}")
"""
"""
print("action space shape", env.action_space.shape)
print("action space sample", )
print("action space sample", env.action_space.sample())
print("action space sample", env.action_space.sample())
print("action space sample", env.action_space.sample())
print("action space sample", env.action_space.sample())
print("observation space shape", env.observation_space.shape)
print("observation space sample", env.observation_space.sample())
member_added = env.bridge.add_member(3, 4, 1, 2, 0, 0, 3)
print("member_added:", member_added)
print(env.bridge.n_joints)
print(len(env.bridge.joints))
print(len(env.bridge.joint_coords))
# print(env.bridge.joint_coords.keys())
print(np.sum(env.bridge.get_state()[0]))

NUM_LOAD_SCENARIOS = 392
# x_values = []
# y_values = []
scenarios_to_check = [7, 105, 203, 301]
# for i in range(NUM_LOAD_SCENARIOS):
for i in scenarios_to_check:
    env = BridgeEnv(i)

    print("Load Scenario:", i)
    print("n_joints:", env.bridge.n_joints)
    print("length of joints list:", len(env.bridge.joints))
    printout = "joints list:"
    for joint in env.bridge.joints:
        printout += f" ({joint.x}, {joint.y}),"
    print(printout)
    print("length of joint_coords:", len(env.bridge.joint_coords))
    print("joint_coords:", env.bridge.joint_coords.keys())
    print("state:", np.sum(env.bridge.get_state()[0]))
    print("support type:", env.bridge.load_scenario.support_type)
    print("under grids:", env.bridge.load_scenario.under_grids)
    print("id:", env.bridge.load_scenario.desc.id)
    print("-------------------")

# x_values.append(env.bridge.load_scenario.grid_size)
# y_values.append(joint.y)

# max_x = max(x_values)
# min_x = min(x_values)
# max_y = max(y_values)
# min_y = min(y_values)

# print('max_x', max_x)
# print('min_x', min_x)
# print('max_y', max_y)
# print('min_y', min_y)


print(env.bridge.load_scenario.desc.id)
print(env.bridge.load_scenario.over_meters)
print(env.bridge.load_scenario.under_meters)
print(env.bridge.joint_coords.keys())
"""

# <DONE!>
# Max Y is 32
# Min Y is -96
# Total Y size is 128
# <DONE!>
# Max X is 208
# Min X is -32
# Total X size is 240

# <DONE!> Max number of Joints from C library is defined as 100, will use 128 as max

# <DONE!> When the Bridge Adj Matrix to real x,y values
#         subtract 32 from X and 96 from Y

# <DONE!> Will need also to reject X and Y values outside of the Load Scenario Min and Max X

# Could also try to figure out how to reject X and Y values that are "in the ground" (referencing the Bridge Designer GUI)

# Apply Convolution layers to the input, since like an image, there will be many zeros in the state tensor
