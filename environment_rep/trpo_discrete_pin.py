from __future__ import print_function
from __future__ import absolute_import

# from rllab.algos.trpo import TRPO
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

class PolicyGenerator:

    """
    Class that trains a tensioning policy given a config file and a file to dump the policy to.
    """

    def __init__(self, experiment_folder="../experiment_data/experiments/2/", config_file="experiment.json", writefile="policydiscrete"):
        self.experiment_folder = experiment_folder
        self.config_file = self.experiment_folder + config_file
        self.writefile = writefile
        self.scorer = Scorer(0)
        self.simulation = load_simulation_from_config(self.config_file)
        self.pin_position, self.option = load_pin_from_config(self.config_file)
        self.simulation.reset()


    def train(self):
        """
        Trains a policy and dumps it to file.
        """
        env = normalize(PinEnvDiscrete(self.simulation, self.pin_position[0], self.pin_position[1], self.simulation.trajectory, 0, self.option))
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
            n_itr = 750
        )
        algo.train()
        with open(self.experiment_folder + self.writefile, "w+") as f:
            pickle.dump(policy, f)


if __name__ == '__main__':
    
    if len(sys.argv) > 1:
        writefile = sys.argv[1]
    else:
        writefile = "policydiscrete.p"
    experiment_folder = "../experiment_data/experiments/4/"
    config_file = "experiment.json"
    pg = PolicyGenerator(experiment_folder, config_file, writefile)
    pg.train()
