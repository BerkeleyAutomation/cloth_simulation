from rllab.envs.base import Env
from rllab.spaces import Discrete
from rllab.spaces import Box
from rllab.envs.base import Step
import numpy as np
import sys, pickle, os
sys.path.append(os.path.dirname(os.getcwd()))
from simulation import *
from scorer import *
from shapecloth import *
from tensioner import *

"""
A Rllab Environment for the tensioning policy experiments.
"""


class PinEnvDiscrete(Env):

    MAPPING = {
        0 : (0,0,0),
        1 : (1,0,0),
        2 : (0,1,0),
        3 : (0,0,1),
        4 : (-1,0,0),
        5 : (0,-1,0),
        6 : (0,0,-1)
    }

    def __init__(self, simulation, x, y, trajectory, scorer=0, max_displacement=False):
        self.simulation = simulation
        height, width = simulation.cloth.initial_params[0]
        self.os_dim = height * width * 5
        self.simulation.reset()
        self.tensioner = self.simulation.pin_position(x, y, max_displacement)
        self.scorer = Scorer(scorer)
        self.trajectory = trajectory
        self.traj_index = 0
        self.pinx, self.piny = x, y

    @property
    def observation_space(self):
        return Box(low=np.array([0, -self.tensioner.max_displacement, -self.tensioner.max_displacement, -self.tensioner.max_displacement] + len(self.simulation.cloth.blobs) * [0, 0, -800]),
            high=np.array([len(self.trajectory) + 1, self.tensioner.max_displacement, self.tensioner.max_displacement, self.tensioner.max_displacement]
                + len(self.simulation.cloth.blobs) * [500, 500, 800]))

    @property
    def action_space(self):
        return Discrete(7)


    @property
    def _state(self):
        centroids = np.ravel(np.array(self.simulation.cloth.centroids)).tolist()
        return np.array([self.traj_index] + list(self.tensioner.displacement) + centroids)
    
    @property
    def _score(self):
        disp = np.linalg.norm(self._state[1])
        score = self.scorer.score(self.simulation.cloth)
        if disp >= self.tensioner.max_displacement - 2:
            score -= 100
        return score


    def reset(self):
        self.simulation.reset()
        self.tensioner = self.simulation.pin_position(self.pinx, self.piny, self.tensioner.max_displacement)
        self.traj_index = 0
        observation = np.copy(self._state)
        return observation

    def step(self, action):
        x, y, z = self.MAPPING[action]
        self.tensioner.tension(x, y, z)
        self.simulation.move_mouse(self.trajectory[self.traj_index][0], self.trajectory[self.traj_index][1])
        reward = self.simulation.update()
        # reward = self._score
        done = self.traj_index >= len(self.trajectory) - 1
        next_observation = np.copy(self._state)
        self.traj_index += 1
        return Step(observation=next_observation, reward=reward, done=done)

    def render(self):
        self.simulation.render_sim()

