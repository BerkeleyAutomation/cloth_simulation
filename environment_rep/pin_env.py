from rllab.envs.base import Env
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

class PinEnv(Env):

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
        return Box(low=np.array([0, -self.tensioner.max_displacement, -self.tensioner.max_displacement, -self.tensioner.max_displacement]),
            high=np.array([len(self.trajectory) + 1, self.tensioner.max_displacement, self.tensioner.max_displacement, self.tensioner.max_displacement]))

    @property
    def action_space(self):
        return Box(low=np.array([-1, -1, -1]), high=np.array([1, 1, 1]))


    @property
    def _state(self):
        return np.array([self.traj_index] + list(self.tensioner.displacement))
    
    @property
    def _score(self):
        return self.scorer.score(self.simulation.cloth)


    def reset(self):
        self.simulation.reset()
        self.tensioner = self.simulation.pin_position(self.pinx, self.piny, self.tensioner.max_displacement)
        self.traj_index = 0
        observation = np.copy(self._state)
        return observation

    def step(self, action):
        x, y, z = action
        self.tensioner.tension(x, y, z)
        self.simulation.move_mouse(self.trajectory[self.traj_index][0], self.trajectory[self.traj_index][1])
        self.simulation.update()
        reward = self._score
        done = self.traj_index >= len(self.trajectory) - 1
        next_observation = np.copy(self._state)
        self.traj_index += 1
        return Step(observation=next_observation, reward=reward, done=done)

    def render(self):
        self.simulation.render_sim()

