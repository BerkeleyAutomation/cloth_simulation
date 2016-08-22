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
sys.path.append(os.path.dirname(os.getcwd()))
from simulation import *
from scorer import *
from shapecloth import *
from tensioner import *

if __name__ == '__main__':
    
    scorer = Scorer(0)
    simulation = load_simulation_from_config("../config_files/experiment.json")
    pin_position = load_pin_from_config("../config_files/experiment.json")
    simulation.reset()
    env = normalize(PinEnv(simulation, pin_position[0], pin_position[1], simulation.trajectory))


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
        batch_size=1000,
        step_size = 0.0001,
        discount = 1,
    )

    # algo = VPG(
    #     env=env,
    #     policy=policy,
    #     baseline=baseline,
    #     batch_size=10
    #     # plot=True,
    # )

    algo.train()

    with open("../experiment_data/policy.p", "w+") as f:
        pickle.dump(policy, f)
