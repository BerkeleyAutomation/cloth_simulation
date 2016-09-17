import matplotlib.pyplot as plt
import numpy as np
import sys, pickle, os, copy
from trpo_discrete_pin import *
from pin_env_discrete import *

if __name__ == '__main__':
    policy = sys.argv[1]
    datafile = sys.argv[2]

    with open(policy, "rb") as f:
        policy = pickle.load(f)

    d = {}
    for i in range(21):
        gravity = -i * 200
        d[gravity] = []
        print gravity
        env = PolicyGenerator("", "experiment.json", gravity=gravity).env
        for j in range(20):
            a, b = rollout(env, policy)
            d[gravity].append([a, sum(b)])

    with open(datafile, "w+") as f:
        pickle.dump(d, f)

