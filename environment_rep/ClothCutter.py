from rlpy.Domains.Domain import Domain
from rlpy.Tools import wrap
import numpy as np
import sys
sys.path.insert(0,'..')
# sys.path.append("/Users/rliaw/research/cloth_simulation/simulation.py")
from simulation import Simulation
from mouse import Mouse
from shapecloth import ShapeCloth
import logging

class ClothCutter(Domain):
	### RLPY Domain 
	### Models the current state of the scissors wrt the cloth, which is not encoded in the state representation


	actions_num = 3
	state_space_dims = 4
	continuous_dims = np.arange(state_space_dims)

	INIT_STATE = np.array([450.0, 300., np.pi/2])
	state_space_dims = 3
	TURN_ANGLE = np.pi / 30

	ANGLE_MIN = np.pi/2
	ANGLE_MAX = 2*np.pi + np.pi/2

	XMIN = 0
	XMAX = 600
	YMIN = 0
	YMAX = 600

	dt = 15.7007

	GOAL_STATE = np.array([300., 150.])
	GOAL_RADIUS = 0.1
	TURN_LEFT, TURN_RIGHT, CUT_FORWARD = range(3)
	episodeCap = None
	_runcount = 0
	prev_reward = []

	def __init__(self):
		self.statespace_limits = np.array(
			[[self.XMIN,
			  self.XMAX],
			 [self.YMIN,
			  self.YMAX],
			 [self.ANGLE_MIN,
			  self.ANGLE_MAX],
			 [0, 100]])
		self.reference_trajectory = [(np.cos(deg) * 150 + 300, np.sin(deg) * 150 + 300) for deg in [ np.pi * i / 30.0 for i in range(61)]]
		shape_fn = lambda x, y: abs((x - 300) **2 + (y - 300) ** 2 - 150 **2) < 2000
		self._mouse = Mouse(down=True)
		self._cloth = ShapeCloth(shape_fn, self._mouse,  width=30, height=30, dx=16, dy=16)
		self.cloth_experiment = Simulation(self._cloth, render=0, init=50)
		self.return_to_goal = False
		self.episodeCap = 60
		logging.getLogger().setLevel(logging.DEBUG)
		super(ClothCutter, self).__init__()

	def s0(self):
		self._stepcount = 0
		self._runcount += 1
		if self._runcount > 100:
			self.cloth_experiment.render = 1
		self.state = np.hstack((self.INIT_STATE.copy(), self._stepcount))
		self.cloth_experiment.reset()
		return self.state.copy(), self.isTerminal(), self.possibleActions()

	def possibleActions(self, s=None):
		return [self.TURN_LEFT, self.TURN_RIGHT, self.CUT_FORWARD]

	# def augmentwithref(self, state):
	# 	return np.hstack

	def step(self, a):

		self._stepcount += 1
		# a = 0
		# if self._runcount % 20 == 0:
		# 	self.cloth_experiment.render = True
		# 	logging.info("%d steps..." % self._stepcount)
		# else:
		# 	self.cloth_experiment.render = False
		x, y, angle = self.state[:3]
		nx, ny, nangle = x, y, angle
		if a == self.TURN_LEFT:
			nangle = angle + self.TURN_ANGLE
		elif a == self.TURN_RIGHT:
			nangle = angle - self.TURN_ANGLE
		# elif a == self.CUT_FORWARD:
		# 	logging.debug("straight")
		nangle = wrap(nangle, self.ANGLE_MIN, self.ANGLE_MAX)
		nx = x + np.cos(angle) * self.dt
		ny = y + np.sin(angle) * self.dt
		# else:
		# 	assert False, "Unknown action taken..."
		## modify environment
		self.cloth_experiment.move_mouse(nx, ny)
		self.cloth_experiment.update()

		ns = np.hstack(([nx, ny, nangle], self._stepcount))
		self.state = ns.copy()
		terminal = self.isTerminal()
		r = self._reward_function(self.state, terminal)
		# logging.debug("Reference: (%d, %d) \t Actual  (%d, %d)" % tuple(np.hstack((self.reference_trajectory[self._stepcount], ns[:2]))))
		logging.debug("Reward: %f" % r)
		return r, ns, terminal, self.possibleActions()

	def _reward_function(self, state, terminal=False):
		assert len(state) == self.state_space_dims
		# 	return -(100 - self._stepcount) * 2 #len(self._cloth.shapepts)
		new_reward = -(np.linalg.norm(state[:2]-  self.reference_trajectory[self._stepcount]) / 50)**2
		if new_reward < -0.2:
			return -1
		# new_reward = -(np.linalg.norm(state[:2]-  self.reference_trajectory[self._stepcount - 1]) / 50)**4
		# self.prev_reward.append(new_reward)
		# if len(self.prev_reward) > 5:
		# 	self.prev_reward.pop(0)
		# return sum(self.prev_reward)
		return new_reward

	def _dynamics(self, a):
		pass

	def _out_of_bounds(self):
		x, y = self.state[:2]
		return x < self.XMIN or x > self.XMAX or x < self.YMIN or x > self.YMAX


	def isTerminal(self):
		if self._out_of_bounds() or self._reward_function(self.state) < -0.2:
			return True
		terminal = np.linalg.norm(self.state[:2] - self.GOAL_STATE) < self.GOAL_RADIUS
		return terminal
		# if terminal and self.return_to_goal:
		# 	self.return_to_goal = False
		# 	return False
		# else:
		# 	return terminal
