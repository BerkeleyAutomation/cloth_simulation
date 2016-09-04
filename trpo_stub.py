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

from environment_rep import *
from environment_rep.pin_env_discrete import *
import numpy as np
import sys, pickle, os
from simulation import *
from scorer import *
from shapecloth import *
from tensioner import *

stub(globals())

experiment_folder="experiment_data/experiments/4/"
config_file="experiment.json"
writefile="policydiscrete.p"
experiment_folder = experiment_folder
config_file = experiment_folder + config_file
writefile = writefile
simulation = load_simulation_from_config(config_file)
pin_position, option = load_pin_from_config(config_file)
simulation.reset()
env = normalize(PinEnvDiscrete(simulation, pin_position[0], pin_position[1], simulation.trajectory, 0, option))
policy = CategoricalMLPPolicy(
    env_spec=env.spec,
    hidden_sizes=(32, 32)
)
baseline = ZeroBaseline(env_spec=env.spec)
algo = TRPO(
    env=env,
    policy=policy,
    baseline=baseline,
    batch_size=1000,
    step_size = 0.001,
    discount = 1,
    n_itr = 500
)

run_experiment_lite(
    algo.train(),
    n_parallel=1,
    snapshot_mode="last",
    log_dir="temp",
    seed=1,
    # plot=True,
)

# algo.train()

with open(experiment_folder + writefile, "w+") as f:
    pickle.dump(policy, f)