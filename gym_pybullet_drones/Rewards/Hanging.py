import numpy as np


def interpolate_distance(distance, max_value, max_reward, min_value=0, min_reward=0):
    return min_reward + ((max_reward - min_reward) * (distance - min_value)) / (max_value - min_value)


def _distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))


class Hanging():
    def reward_fun(self, state, has_collided, dist_tether_branch, dist_drone_branch, num_wraps):
        x, y, z = state

        if z < 2.5 and z > 0.3 and x > -0.5 and x < 0.5:
            return 1.0, True, None
        else:
            if x > 0:
                return 0, False, None
            else:
                return max(0.0, 1.0 - 0.2 * _distance([x, z], [0, 1.5]) * _distance([x, z], [0, 1.5])), False, None
