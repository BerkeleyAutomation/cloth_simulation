from __future__ import absolute_import
import sys, pickle, os

from rllab.algos.trpo import TRPO
from rllab.algos.vpg import VPG
from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
from rllab.baselines.zero_baseline import ZeroBaseline
from rllab.envs.gym_env import GymEnv
from rllab.envs.normalized_env import normalize
from rllab.misc.instrument import stub, run_experiment_lite
from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy
from rllab.policies.categorical_mlp_policy import CategoricalMLPPolicy
from os.path import dirname
sys.path.append(dirname(dirname(dirname(os.getcwd()))))
from pin_env_discrete import *
import numpy as np
from simulation import *
from scorer import *
from shapecloth import *
from tensioner import *
from trpo_discrete_pin import *

if __name__ == '__main__':
    policy = "circle/five.p"
    datafile = "circle/test5_5.p"
    policy = pickle.load(open(policy, "rb"))
    env = PolicyGenerator(None, "", "circle/experiment4.json", "", "").env

    scores = []
    for i in range(10):
        score1, score2 = rollout(env, policy)
        scores.append([score1, score2])
        print score1, score2
    # pickle.dump(scores, open(datafile, "w+"))
