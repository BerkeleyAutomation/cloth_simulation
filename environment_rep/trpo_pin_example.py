from __future__ import print_function
from __future__ import absolute_import

from rllab.algos.trpo import TRPO
from rllab.algos.vpg import VPG
from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
from rllab.baselines.zero_baseline import ZeroBaseline
from rllab.envs.gym_env import GymEnv
from rllab.envs.normalized_env import normalize
from rllab.misc.instrument import stub, run_experiment_lite
from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy

from pin_env import *
import numpy as np
import sys, pickle, os
sys.path.append("/home/davinci0/cloth_simulation")
from simulation import *
from scorer import *
from shapecloth import *
from tensioner import *

if __name__ == '__main__':
	
	shape_fn = lambda x, y: abs((x - 300) **2 + (y - 300) ** 2 - 150 **2) < 2000
	scorer = Scorer(0)
	simulation = load_simulation_from_config("../config_files/default.json", shape_fn)
	simulation.reset()
	trajectory = [(np.cos(deg) * 150 + 300, np.sin(deg) * 150 + 300) for deg in [3.6 * np.pi * i / 180.0 for i in range(100)]]

	# stub(globals())

	env = normalize(PinEnv(simulation, 300, 300, trajectory))


	policy = GaussianMLPPolicy(
	    env_spec=env.spec,
	    # The neural network policy should have two hidden layers, each with 32 hidden units.
	    hidden_sizes=(32, 32)
	)

	# baseline = LinearFeatureBaseline(env_spec=env.spec)
	baseline = ZeroBaseline(env_spec=env.spec)

	algo = TRPO(
	    env=env,
	    policy=policy,
	    baseline=baseline,
	    batch_size=100,
	    step_size = 0.01
	    # Uncomment both lines (this and the plot parameter below) to enable plotting
	    # plot=True,
	)

	# algo = VPG(
	#     env=env,
	#     policy=policy,
	#     baseline=baseline,
	#     batch_size=10
	#     # Uncomment both lines (this and the plot parameter below) to enable plotting
	#     # plot=True,
	# )

	algo.train()