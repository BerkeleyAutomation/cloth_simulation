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
from rllab.policies.categorical_mlp_policy import CategoricalMLPPolicy

from pin_env_discrete import *
import numpy as np
import sys, pickle, os
sys.path.append(os.path.dirname(os.getcwd()))
from simulation import *
from scorer import *
from shapecloth import *
from tensioner import *

if __name__ == '__main__':
    
    if len(sys.argv) > 1:
        writefile = sys.argv[1]
    else:
        writefile = "policydiscrete.p"
    experiment_folder = "../experiment_data/experiments/0/"
    config_file = experiment_folder + "experiment.json"
    writefile = experiment_folder + writefile
    scorer = Scorer(0)
    simulation = load_simulation_from_config(config_file)
    simulation.trajectory = simulation.trajectory[::-1]
    pin_position, option = load_pin_from_config(config_file)
    simulation.reset()
    env = normalize(PinEnvDiscrete(simulation, pin_position[0], pin_position[1], simulation.trajectory, 0, option))


    policy = CategoricalMLPPolicy(
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
        step_size = 0.001,
        discount = 1,
        n_itr = 750
    )

    # algo = VPG(
    #     env=env,
    #     policy=policy,
    #     baseline=baseline,
    #     batch_size=10
    #     # plot=True,
    # )

    algo.train()

    with open("../experiment_data/" + writefile, "w+") as f:
        pickle.dump(policy, f)
