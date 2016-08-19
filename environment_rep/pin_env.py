from rllab.envs.base import Env
from rllab.spaces import Box
from rllab.envs.base import Step
import numpy as np
import sys, pickle, os
sys.path.append("/home/davinci0/cloth_simulation")
from simulation import *
from scorer import *
from shapecloth import *
from tensioner import *

"""
A Rllab Environment for the tensioning policy experiments.
"""

class PinEnv(Env):

    def __init__(self, simulation, x, y, trajectory, scorer=1):
        self.simulation = simulation
        height, width = simulation.cloth.initial_params[0]
        self.os_dim = height * width * 5
        self.simulation.reset()
        self.tensioner = self.simulation.pin_position(x, y)
        self.scorer = Scorer(scorer)
        self.trajectory = trajectory
        self.traj_index = 0

    @property
    def observation_space(self):
        return Box(low=-800, high=800, shape=(self.os_dim + 5,))

    @property
    def action_space(self):
        return Box(low=-10, high=10, shape=(3,))

    @property
    def _state(self):
        lst = []
        for i in range(self.os_dim / 5):
            pt = self.simulation.cloth.allpts[i]
            lst.append([pt.x, pt.y, pt.z, len(pt.constraints), pt.shape])
        return np.array(lst + [self.tensioner.x, self.tensioner.y] + list(self.tensioner.displacement))

    @property
    def _score(self):
        return self.scorer.score(self.simulation.cloth)

    def reset(self):
        self.simulation.reset()
        observation = np.copy(self._state)
        return observation

    def step(self, action):
        x, y, z = action
        self.tensioner.tension(x, y, z)
        self.simulation.move_mouse(self.trajectory[self.traj_index][0], self.trajectory[self.traj_index][1])
        self.simulation.update()
        reward = self._score
        done = reward == 0 or self.traj_index >= len(self.trajectory) - 1
        next_observation = np.copy(self._state)
        self.traj_index += 1
        return Step(observation=next_observation, reward=reward, done=done)

    def render(self):
        self.simulation.render_sim()