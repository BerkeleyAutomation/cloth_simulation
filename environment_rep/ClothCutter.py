from rlpy.Domains.Domain import Domain
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
	state_space_dims = 3
	continuous_dims = np.arange(state_space_dims)

	INIT_STATE = np.array([450.0, 300., np.pi/2])
	state_space_dims = 3
	TURN_ANGLE = np.pi / 6

	ANGLE_MIN = -np.pi
	ANGLE_MAX = np.pi

	XMIN = 0
	XMAX = 600
	YMIN = 0
	YMAX = 600

	dt = 4

	GOAL_STATE = np.array([300., 150.])
	GOAL_RADIUS = 0.1
	TURN_LEFT, TURN_RIGHT, CUT_FORWARD = range(3)


	def __init__(self):
		self.statespace_limits = np.array(
			[[self.XMIN,
			  self.XMAX],
			 [self.YMIN,
			  self.YMAX],
				[self.ANGLE_MIN,
				 self.ANGLE_MAX]])
		shape_fn = lambda x, y: abs((x - 300) **2 + (y - 300) ** 2 - 150 **2) < 2000
		self._mouse = Mouse(down=True)
		self._cloth = ShapeCloth(shape_fn, self._mouse)
		self.cloth_experiment = Simulation(self._cloth, render=True, init=50)
		self.return_to_goal = False
		self.episodeCap = 180
		super(ClothCutter, self).__init__()

	def s0(self):
		self.state = self.INIT_STATE.copy()
		self.cloth_experiment.reset()

	def possibleActions(self, s=None):
		return [self.TURN_LEFT, self.TURN_RIGHT, self.CUT_FORWARD]

	def step(self, a):
		x, y, angle = self.state
		nx, ny, nangle = x, y, angle
		if a == self.TURN_LEFT:
			nangle = angle - self.TURN_ANGLE if self.ANGLE_MIN <= angle - self.TURN_ANGLE  else angle
		elif a == self.TURN_RIGHT:
			nangle = angle + self.TURN_ANGLE if self.ANGLE_MAX >= angle + self.TURN_ANGLE  else angle
		elif a == self.CUT_FORWARD:
			nx = x + np.cos(angle) * self.dt
			ny = y + np.sin(angle) * self.dt
		else:
			assert False, "Unknown action taken..."
		## modify environment
		self.cloth_experiment.move_mouse(nx, ny)
		self.cloth_experiment.update()

		ns = np.array([nx, ny, nangle])
		self.state = ns.copy()
		terminal = self.isTerminal()
		r = self._reward_function(self.state)
		return r, ns, terminal, self.possibleActions()

	def _reward_function(self, state):
		return - len(self.cloth_experiment.cloth.shapepts)

	def _dynamics(self, a):
		pass

	def isTerminal(self):
		terminal = np.linalg.norm(self.state[:2] - self.GOAL_STATE) < self.GOAL_RADIUS

		if terminal and self.return_to_goal:
			self.return_to_goal = False
			return False
		else:
			return terminal
