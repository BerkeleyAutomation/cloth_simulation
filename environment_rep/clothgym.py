import math
import gym
from gym import spaces

from gym.utils import seeding
import numpy as np
import sys
# sys.path.insert(0,'..')
sys.path.append("/home/davinci0/cloth_simulation/")
from simulation import Simulation
from mouse import Mouse
from shapecloth import ShapeCloth
import logging


class ClothEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 30
    }

    TURN_ANGLE = np.pi / 15

    ANGLE_MIN = np.pi/2
    ANGLE_MAX = 2*np.pi + np.pi/2

    XMIN = 0
    XMAX = 600
    YMIN = 0
    YMAX = 600
    TURN_LEFT, TURN_RIGHT, CUT_FORWARD = range(3)

    dt = 15.7007
    _stepcount = 0

    def __init__(self):
        self.viewer = None

        self.low = np.array([self.XMIN, self.YMIN, self.ANGLE_MIN, 0])
        self.high = np.array([self.XMAX, self.YMAX, self.ANGLE_MAX, 100])

        self.action_space = spaces.Box(np.array([-self.TURN_ANGLE]), np.array([self.TURN_ANGLE]))
        self.observation_space = spaces.Box(self.low, self.high)

        self._reference_trajectory = [(np.cos(deg) * 150 + 300, np.sin(deg) * 150 + 300) for deg in [ np.pi * i / 30.0 for i in range(61)]]
        shape_fn = lambda x, y: abs((x - 300) **2 + (y - 300) ** 2 - 150 **2) < 2000
        self._mouse = Mouse(down=True)
        self._cloth = ShapeCloth(shape_fn, self._mouse,  width=30, height=30, dx=16, dy=16)
        self.cloth_experiment = Simulation(self._cloth, render=0, init=50)

        self._seed()
        self.reset()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):
        self._stepcount += 1
        x, y, angle = self.state[:3]
        nx, ny, nangle = x, y, angle
        # if action == self.TURN_LEFT:
        #     nangle = angle + self.TURN_ANGLE
        # elif action == self.TURN_RIGHT:
        nangle = angle + action
        nangle = wrap(nangle, self.ANGLE_MIN, self.ANGLE_MAX)
        nx = x + np.cos(angle) * self.dt
        ny = y + np.sin(angle) * self.dt

        ### MODIFY CLOTH ###
        self.cloth_experiment.move_mouse(nx, ny)
        self.cloth_experiment.update()

        ns = np.hstack(([nx, ny, nangle], self._stepcount))
        self.state = ns.copy()
        terminal = self._terminal()
        r = self._reward_function(self.state, terminal)

        return np.array(self.state), r, terminal, {}

    def _reset(self):
        self._stepcount = 0
        self.state = np.array([450.0, 300., np.pi/2, 0])
        self.cloth_experiment.reset()
        return np.array(self.state)

    def _terminal(self):
        if self._out_of_bounds() or self._reward_function(self.state) < -1.:
            return True
        # terminal = np.linalg.norm(self.state[:2] - self.GOAL_STATE) < self.GOAL_RADIUS
        terminal = self._stepcount == 60
        return terminal

    def _reward_function(self, state, terminal=False):
        #   return -(100 - self._stepcount) * 2 #len(self._cloth.shapepts)
        new_reward = -(np.linalg.norm(state[:2]-  self._reference_trajectory[self._stepcount]) / 50)**2
        if new_reward < -0.2:
            return -1
        # new_reward = -(np.linalg.norm(state[:2]-  self.reference_trajectory[self._stepcount - 1]) / 50)**4
        # self.prev_reward.append(new_reward)
        # if len(self.prev_reward) > 5:
        #   self.prev_reward.pop(0)
        # return sum(self.prev_reward)
        return new_reward

    def _out_of_bounds(self):
        x, y = self.state[:2]
        return x < self.XMIN or x > self.XMAX or x < self.YMIN or x > self.YMAX


    # def _render(self, mode='human', close=False):
    #     if close:
    #         if self.viewer is not None:
    #             self.viewer.close()
    #             self.viewer = None
    #         return

    #     screen_width = 600
    #     screen_height = 400

    #     world_width = self.max_position - self.min_position
    #     scale = screen_width/world_width
    #     carwidth=40
    #     carheight=20


    #     if self.viewer is None:
    #         from gym.envs.classic_control import rendering
    #         self.viewer = rendering.Viewer(screen_width, screen_height)
    #         xs = np.linspace(self.min_position, self.max_position, 100)
    #         ys = self._height(xs)
    #         xys = list(zip((xs-self.min_position)*scale, ys*scale))

    #         self.track = rendering.make_polyline(xys)
    #         self.track.set_linewidth(4)
    #         self.viewer.add_geom(self.track)

    #         clearance = 10

    #         l,r,t,b = -carwidth/2, carwidth/2, carheight, 0
    #         car = rendering.FilledPolygon([(l,b), (l,t), (r,t), (r,b)])
    #         car.add_attr(rendering.Transform(translation=(0, clearance)))
    #         self.cartrans = rendering.Transform()
    #         car.add_attr(self.cartrans)
    #         self.viewer.add_geom(car)
    #         frontwheel = rendering.make_circle(carheight/2.5)
    #         frontwheel.set_color(.5, .5, .5)
    #         frontwheel.add_attr(rendering.Transform(translation=(carwidth/4,clearance)))
    #         frontwheel.add_attr(self.cartrans)
    #         self.viewer.add_geom(frontwheel)
    #         backwheel = rendering.make_circle(carheight/2.5)
    #         backwheel.add_attr(rendering.Transform(translation=(-carwidth/4,clearance)))
    #         backwheel.add_attr(self.cartrans)
    #         backwheel.set_color(.5, .5, .5)
    #         self.viewer.add_geom(backwheel)
    #         flagx = (self.goal_position-self.min_position)*scale
    #         flagy1 = self._height(self.goal_position)*scale
    #         flagy2 = flagy1 + 50
    #         flagpole = rendering.Line((flagx, flagy1), (flagx, flagy2))
    #         self.viewer.add_geom(flagpole)
    #         flag = rendering.FilledPolygon([(flagx, flagy2), (flagx, flagy2-10), (flagx+25, flagy2-5)])
    #         flag.set_color(.8,.8,0)
    #         self.viewer.add_geom(flag)

    #     pos = self.state[0]
    #     self.cartrans.set_translation((pos-self.min_position)*scale, self._height(pos)*scale)
    #     self.cartrans.set_rotation(math.cos(3 * pos))

    #     return self.viewer.render(return_rgb_array = mode=='rgb_array')

def wrap(x, m, M):
    """
    :param x: a scalar
    :param m: minimum possible value in range
    :param M: maximum possible value in range
    Wraps ``x`` so m <= x <= M; but unlike ``bound()`` which
    truncates, ``wrap()`` wraps x around the coordinate system defined by m,M.\n
    For example, m = -180, M = 180 (degrees), x = 360 --> returns 0.
    """
    diff = M - m
    while x > M:
        x = x - diff
    while x < m:
        x = x + diff
    return x
