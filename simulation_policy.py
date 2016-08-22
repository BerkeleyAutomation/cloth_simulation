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

    policy = load_policy("experiment_data/policy.p")
    scorer = Scorer(0)
    simulation = load_simulation_from_config(shape_fn=shape_fn)
    simulation.reset()

    simulation.render = True


    print "Initial Score", scorer.score(simulation.cloth)

    trajectory = [(np.cos(deg) * 150 + 300, np.sin(deg) * 150 + 300) for deg in [3.6 * np.pi * i / 180.0 for i in range(100)]]

    tensioner = simulation.pin_position(300, 300)

    for i in range(100):
        simulation.update()
        action = query_policy(policy, i)
        print action
        tensioner.tension(action[1]['mean'][0], action[1]['mean'][1])
        simulation.move_mouse(trajectory[i][0], trajectory[i][1])    

    print "Score", scorer.score(simulation.cloth)
