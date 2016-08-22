import numpy as np
import matplotlib.pyplot as plt
import pickle, copy, sys
import json
from cloth import *
from circlecloth import *
from shapecloth import *
from tensioner import *
from mouse import *
from registration import *
from scorer import *

"""
Presents a visualization of a learned policy.
"""

def load_policy(fname):
    with open(fname, "rb") as f:
        try:
            return pickle.load(f)
        except EOFError:
            print 'Nothing written to file.'

def query_policy(policy, observation):
    observation = np.array(observation)
    return policy.get_action(observation)

if __name__ == "__main__":
    if len(sys.argv) <= 1:
        shape_fn = lambda x, y: abs((x - 300) **2 + (y - 300) ** 2 - 150 **2) < 2000
    else:
        shape_fn=None

    simulation = load_simulation_from_config("config_files/experiment.json", shape_fn)
    policy = load_policy("experiment_data/policy.p")
    scorer = Scorer(0)
    simulation.reset()
    simulation.render = True

    print "Initial Score", scorer.score(simulation.cloth)

    tensioner = simulation.pin_position(300, 300)

    for i in range(len(simulation.trajectory)):
        simulation.update()
        action = query_policy(policy, i)
        print action[1]['mean']
        tensioner.tension(action[1]['mean'][0], action[1]['mean'][1])
        simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])    

    print "Score", scorer.score(simulation.cloth)
