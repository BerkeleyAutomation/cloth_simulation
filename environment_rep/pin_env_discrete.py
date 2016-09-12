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

    def __init__(self, simulation, x, y, trajectory, scorer=0, max_displacement=False, predict=False, original=False):
        self.simulation = simulation
        height, width = simulation.cloth.initial_params[0]
        self.os_dim = height * width * 5
        self.simulation.reset()
        self.tensioner = self.simulation.pin_position(x, y, max_displacement)
        self.scorer = Scorer(scorer)
        self.trajectory = trajectory
        self.traj_index = 0
        self.pinx, self.piny = x, y
        self.predict = predict
        self.original = original
        self.last_score = 0

    @property
    def observation_space(self):
        if self.original:
            return Box(low=np.array([0, -self.tensioner.max_displacement, -self.tensioner.max_displacement, -self.tensioner.max_displacement]),
                high=np.array([len(self.trajectory) + 1, self.tensioner.max_displacement, self.tensioner.max_displacement, self.tensioner.max_displacement]))
        if not self.predict:
            return Box(low=np.array([0, -self.tensioner.max_displacement, -self.tensioner.max_displacement, -self.tensioner.max_displacement] + len(self.simulation.cloth.blobs) * [0, 0, -800]),
                high=np.array([len(self.trajectory) + 1, self.tensioner.max_displacement, self.tensioner.max_displacement, self.tensioner.max_displacement]
                    + len(self.simulation.cloth.blobs) * [500, 500, 800]))
        return Box(low=np.array([0, -self.tensioner.max_displacement, -self.tensioner.max_displacement, -self.tensioner.max_displacement] + len(self.simulation.cloth.blobs) * [0, 0, -800] + 3 * [-1000, -1000, -1000, -1000, -3.2] + [0, 0]),
            high=np.array([len(self.trajectory) + 1, self.tensioner.max_displacement, self.tensioner.max_displacement, self.tensioner.max_displacement]
                + len(self.simulation.cloth.blobs) * [500, 500, 800] + 3 * [800, 800, 800, 800, 3.2] + [600, 600]))

    @property
    def action_space(self):
        return Discrete(7)


    @property
    def _state(self):
        scissors = self.simulation.mouse.x, self.simulation.mouse.y
        centroids = np.ravel(np.array(self.simulation.cloth.centroids)).tolist()
        if self.original:
            return np.array([self.traj_index] + list(self.tensioner.displacement))
        if not self.predict:
            return np.array([self.traj_index] + list(self.tensioner.displacement) + centroids)
        next_position3 = [-1000, -1000]
        closest_shape3 = [-1000, -1000]
        angle3 = 0
        next_position2 = [-1000, -1000]
        closest_shape2 = [-1000, -1000]
        angle2 = 0
        next_position = [-1000, -1000]
        closest_shape = [-1000, -1000]
        angle = 0
        if self.traj_index < len(self.trajectory) - 1:
            next_position = [self.trajectory[self.traj_index+1][0], self.trajectory[self.traj_index+1][1]]
            closest_shape = list(self.simulation.cloth.find_closest_shapept(next_position[0], next_position[1]))
            angle = self.simulation.cloth.find_dtheta(scissors[0], scissors[1], next_position[0], next_position[1], closest_shape[0], closest_shape[1])
            if self.traj_index < len(self.trajectory) - 5:
                next_position2 = [self.trajectory[self.traj_index+5][0], self.trajectory[self.traj_index+5][1]]
                if np.linalg.norm(np.array(next_position2) - np.array(next_position)) < 100:
                    closest_shape2 = list(self.simulation.cloth.find_closest_shapept(next_position2[0], next_position2[1]))
                    angle2 = self.simulation.cloth.find_dtheta(next_position[0], next_position[1], next_position2[0], next_position2[1], closest_shape2[0], closest_shape2[1])
                    if self.traj_index < len(self.trajectory) - 10:
                        next_position3 = [self.trajectory[self.traj_index+10][0], self.trajectory[self.traj_index+10][1]]
                        if np.linalg.norm(np.array(next_position3) - np.array(next_position2)) < 100:
                            closest_shape3 = list(self.simulation.cloth.find_closest_shapept(next_position3[0], next_position3[1]))
                            angle3 = self.simulation.cloth.find_dtheta(next_position2[0], next_position2[1], next_position3[0], next_position3[1], closest_shape3[0], closest_shape3[1])
        return np.array([self.traj_index] + list(self.tensioner.displacement) + centroids + next_position + closest_shape + [angle] + next_position2 + closest_shape2 + [angle2]
            + next_position3 + closest_shape3 + [angle3] + list(scissors))
    
    @property
    def _score(self):
        disp = np.linalg.norm(self._state[1])
        score = self.scorer.score(self.simulation.cloth)
        if disp >= self.tensioner.max_displacement - 2:
            score -= 100
        return score


    def reset(self):
        self.last_score = 0
        self.simulation.reset()
        self.tensioner = self.simulation.pin_position(self.pinx, self.piny, self.tensioner.max_displacement)
        self.traj_index = 0
        observation = np.copy(self._state)
        return observation

    def step(self, action):
        x, y, z = self.MAPPING[action]
        self.tensioner.tension(x, y, z)
        self.simulation.move_mouse(self.trajectory[self.traj_index][0], self.trajectory[self.traj_index][1])
        # reward = self.simulation.update() * np.ceil(self.traj_index/30)
        self.simulation.update()
        self.traj_index += 1
        self.simulation.move_mouse(self.trajectory[self.traj_index][0], self.trajectory[self.traj_index][1])
        # reward += self.simulation.update() * np.ceil(self.traj_index/30)
        self.simulation.update()
        done = self.traj_index >= len(self.trajectory) - 2
        if done:
            reward = self.simulation.cloth.evaluate()
            print "score", reward
        else:
            reward = 0
        next_observation = np.copy(self._state)
        self.traj_index += 1
        return Step(observation=next_observation, reward=reward, done=done)

    def render(self):
        self.simulation.render_sim()

    # def local_angles(self, n=5):
    #     if self.
    #     for i in range(n):

