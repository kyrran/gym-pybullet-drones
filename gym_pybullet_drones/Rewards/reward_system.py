from gym_pybullet_drones.Rewards.Approaching import CircularApproachingReward
from gym_pybullet_drones.Rewards.Hanging import Hanging
# import matplotlib.pyplot as plt


class RewardSystem():
    def __init__(self, phase: str) -> None:
        self.phase = phase
        print("Phase: ", phase)
        self.approaching_reward = CircularApproachingReward()
        self.hanging_reward = Hanging()

        self.approaching_rewards = []
        self.wrapping_rewards = []
        self.hanging_rewards = []
        self.distance_rewards = []
        self.total_rewards = []

    def calculate(self, state, has_collided, dist_tether_branch, dist_drone_branch,
                  num_wraps):

        # Approaching reward is between -1 and 0
        approaching_reward, _, _ = self.approaching_reward.reward_fun(state, has_collided, dist_tether_branch,
                                                                      dist_drone_branch, num_wraps)
        approaching_reward = approaching_reward if num_wraps < 0.9 else 0.0
        assert approaching_reward <= 0 and approaching_reward >= -1, f"was {approaching_reward}"

        # Wrapping reward is between -1 and 0
        wrapping_reward = 1.0 * min(num_wraps, 1.0) - 1.0
        assert wrapping_reward <= 0 and wrapping_reward >= -1, f"was {wrapping_reward}"

        # Hanging reward is between 0 and 1
        hanging_reward, hanging_done, _ = self.hanging_reward.reward_fun(state, has_collided, dist_tether_branch,
                                                                         dist_drone_branch, num_wraps)
        hanging_reward = hanging_reward - 1  # Now its between -1 and 0
        assert hanging_reward <= 0 and hanging_reward >= -1, f"was {hanging_reward}"

        distance_reward = min(0.5, max(0, dist_drone_branch if num_wraps > 1.0 else 0.0)) - 0.5
        assert distance_reward <= 0 and distance_reward >= -0.5, f"was {distance_reward}"

        total_reward = 0
        match self.phase:
            case "all":
                if num_wraps > 0.75:
                    total_reward = 0 + wrapping_reward + hanging_reward + distance_reward  # Between -2 and 0
                    done = hanging_done
                else:
                    total_reward = approaching_reward + wrapping_reward - 1.5  # Between -3 and -1
                    done = False

            case "approaching":
                total_reward = approaching_reward + wrapping_reward - 1.5  # Between -3 and -1
                done = bool(num_wraps > 0.9)
            case _:
                raise ValueError("Unknown phase type")

        # self.approaching_rewards.append(approaching_reward)
        # self.wrapping_rewards.append(wrapping_reward)
        # self.hanging_rewards.append(hanging_reward)
        # self.distance_rewards.append(distance_reward)
        # self.total_rewards.append(total_reward)

        return total_reward, done

    def reset(self):
        pass
        # plt.figure(figsize=(14, 7))
        # plt.plot(self.approaching_rewards, label='Approaching Reward', marker='o')
        # plt.plot(self.wrapping_rewards, label='Wrapping Reward', marker='o')
        # plt.plot(self.hanging_rewards, label='Hanging Reward', marker='o')
        # plt.plot(self.distance_rewards, label='Distance Reward', marker='o')
        # plt.plot(self.total_rewards, label='Total Reward', marker='o', linestyle='--')

        # plt.xlabel('Time Steps')
        # plt.ylabel('Reward Value')
        # plt.title('Reward Components Over Time')
        # plt.legend()
        # plt.grid(True)
        # plt.show()
