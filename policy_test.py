import sys, pickle, os
import os.path as osp
from trpo_discrete_pin import *

def test_policy(experiment_directory="experiment_data/experiments/4/", config_file="experiment.json", policy="one.p"):
    with open(osp.join(experiment_directory, policy)) as f:
        policy = pickle.load(f)
    pg = PolicyGenerator(experiment_directory, config_file)
    env = pg.env
    rollout(env, policy)
    return env.simulation

if __name__ == '__main__':
    policy = sys.argv[1]
    experiment_directory = "experiment_data/experiments/4/"
    config_file = "experiment.json"
    with open(osp.join(experiment_directory, policy)) as f:
        policy = pickle.load(f)
    pg = PolicyGenerator(experiment_directory, config_file)
    env = pg.env

    rollout(env, policy)

