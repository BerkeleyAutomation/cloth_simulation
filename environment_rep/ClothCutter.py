from rlpy.Domains import Domain
import numpy as np
import sys
sys.path.append("/Users/rliaw/research/cloth_simulation/simulation.py")
from simulation import Simulation

class ClothCutter(Domain):
	### RLPY Domain 
	### Models the current state of the scissors wrt the cloth, which is not encoded in the state representation


    actions_num = 3
    state_space_dims = 3
    continuous_dims = np.arange(state_space_dims)

	INIT_STATE = np.array([])
    state_space_dims = 3
    TURN_ANGLE = np.pi / 6

    ANGLE_MIN = -np.pi
    ANGLE_MAX = np.pi
    dt = 0.1

    GOAL_STATE = np.array([])
    GOAL_RADIUS = 0.1
    TURN_LEFT, TURN_RIGHT, CUT_FORWARD = range(3)


	def __init__(self, **kwargs):
        self.statespace_limits = np.array(
            [[self.XMIN,
              self.XMAX],
             [self.YMIN,
              self.YMAX],
                [self.SPEEDMIN,
                 self.SPEEDMAX],
                [self.HEADINGMIN,
                 self.HEADINGMAX]])
		self.cloth_experiment = Simulation()
		self.return_to_goal = False
		self.episodeCap = 180
		super(ClothCutter, self).__init__(kwargs)

	def s0(self):
		self.state = self.INIT_STATE.copy()
		self.cloth_experiment.reset()

	def possibleActions(self, state):
		return [TURN_LEFT, TURN_RIGHT, CUT_FORWARD]

    def step(self, a):
    	x, y, angle = self.state
    	nx, ny, nangle = x, y, angle
    	if a == TURN_LEFT:
    		nangle = angle - TURN_ANGLE if ANGLE_MIN <= angle - TURN_ANGLE  else angle
    	elif a == TURN_RIGHT:
    		nangle = angle + TURN_ANGLE if ANGLE_MAX >= angle + TURN_ANGLE  else angle
    	elif a == CUT_FORWARD:
    		nx = x + np.cos(angle) * self.dt
    		ny = y + np.sin(angle) * self.dt
    	else:
    		assert False, "Unknown action taken..."

		ns = np.array([nx, ny, nangle])
		self.state = ns.copy()
    	terminal = self.isTerminal()
    	r = - len(self.cloth_experiment.cloth.shapepts)
    	return r, ns, terminal, self.possibleActions()

    def _dynamics(self, a):
    	pass

    def isTerminal(self):
    	terminal = np.linalg.norm(self.state[:2] - self.GOAL_STATE) < self.GOAL_RADIUS

    	if terminal and self.return_to_goal:
    		self.return_to_goal = False
    		return False
    	else:
    		return terminal
