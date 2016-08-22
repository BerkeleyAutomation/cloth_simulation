import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import sys
# sys.path.insert(0,'..')
sys.path.append("/home/davinci0/cloth_simulation/")
from simulation import *
from mouse import *
from shapecloth import *
import logging

"""
An OpenAI Gym environment for the tensioning policy experiments.
"""

### NOTE: code not ready for use yet.


class PinClothEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 30
    }



    TENSIONX_MAX = 10
    TENSIONY_MAX = 10
    TENSIONZ_MAX = 10
    TENSIONX_MIN = -10
    TENSIONY_MIN = -10
    TENSIONZ_MIN = -10

    TENSIONX = 300
    TENSIONY = 300

    SCORER = 1

    dt = 15.7007
    _stepcount = 0

    def __init__(self):
        self.viewer = None

        self.trajectory = read_trajectory_from_file("../experiment_data/trajectory.p")
        self.simulation = load_simulation_from_config(fname="config_files/experiment.json")
        self.scorer = Scorer(SCORER)
        self.mouse = self.simulation.mouse
        self.cloth = self.simulation.cloth
        self.os_dim = height * width * 5

        self.low = np.array([self.TENSIONX_MIN, self.TENSIONY_MIN, self.TENSIONZ_MIN])
        self.high = np.array([self.TENSIONX_MAX, self.TENSIONY_MAX, self.TENSIONZ_MAX])

        obslow = []
        obshigh = []
        for i in range(self.os_dim/5):
            obslow = obslow + [0, 0, -self.simulation.bounds[2], 0, 0]
            obshigh = obshigh + [self.simulation.bounds[0], self.simulation.bounds[1], self.simulation.bounds[2], 2, 1]

        self.obslow = np.array(obslow + [0, 0, -self.simulation.bounds[0], -self.simulation.bounds[1], -self.simulation.bounds[2]])
        self.obshigh = np.array(obshigh + [self.simulation.bounds[0], self.simulation.bounds[1], self.simulation.bounds[0], self.simulation.bounds[1], self.simulation.bounds[2]])

        self.action_space = spaces.Box(self.low, self.high)
        self.observation_space = spaces.Box(self.obslow, self.obshigh)

        self._seed()
        self.reset()
        self.tensioner = self.simulation.pin_position(TENSIONX, TENSIONY)

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):
        x, y, z = action
        self.tensioner.tension(x, y, z)
        self.simulation.move_mouse(self.trajectory[self.traj_index][0], self.trajectory[self.traj_index][1])
        self.simulation.update()
        terminal = self._terminal()
        r = self._reward_function(self.state, terminal)
        self._stepcount += 1
        return np.array(self.state), r, terminal, {}

    def _reset(self):
        self._stepcount = 0
        self.simulation.reset()
        self.mouse = self.simulation.mouse
        self.cloth = self.simulation.cloth
        self.tensioner = self.simulation.pin_position(TENSIONX, TENSIONY)
        return np.array(self.state)

    def _terminal(self):
        return reward == 0 or self._stepcount >= len(self.trajectory) - 1 or _out_of_bounds()

    def _reward_function(self, state, terminal=False):
        return self.score

    def _out_of_bounds(self):
        pts = self.state[:self.os_dim]
        ptsx = pts[::5]
        ptsy = pts[1::5]
        ptsz = pts[2::5]
        flag = False
        if np.max(ptsx) > self.simulation.bounds[0] or np.min(ptsx) < 0 or np.max(ptsy) > self.simulation.bounds[1] or np.min(ptsy) < 0 or np.max(ptsz) > self.simulation.bounds[2] or np.min(ptsz) < -self.simulation.bounds[2]:
            flag = True
        return flag

    @property
    def state(self):
        lst = []
        for i in range(self.os_dim / 5):
            pt = self.simulation.cloth.allpts[i]
            lst.append([pt.x, pt.y, pt.z, len(pt.constraints), pt.shape])
        return np.array(lst + [self.tensioner.x, self.tensioner.y] + list(self.tensioner.displacement))

    @property
    def score(self):
        return self.scorer.score(self.simulation.cloth)

    def _render(self, mode="human", close=False):
        self.simulation.render_sim()
