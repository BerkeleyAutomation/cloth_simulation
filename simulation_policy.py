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

    simulation = load_simulation_from_config("config_files/experiment.json")
    policy = load_policy("experiment_data/policy.p")
    scorer = Scorer(0)
    simulation.reset()
    simulation.render = True
    simulation.trajectory = simulation.trajectory[::-1]

    print "Initial Score", scorer.score(simulation.cloth)

    for i in range(len(simulation.trajectory)):
        simulation.update()
        simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])    

    print "No Pin Score", scorer.score(simulation.cloth)

    simulation.reset()
    pin_position, option = load_pin_from_config("config_files/experiment.json")
    tensioner = simulation.pin_position(pin_position[0], pin_position[1], option)

    for i in range(len(simulation.trajectory)):
        simulation.update()
        simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])    

    print "Fixed Pin Score", scorer.score(simulation.cloth)

    simulation.reset()
    tensioner = simulation.pin_position(pin_position[0], pin_position[1], option)

    for i in range(len(simulation.trajectory)):
        simulation.update()
        action = query_policy(policy, [i]+list(tensioner.displacement))
        print action[1]['mean']
        tensioner.tension(action[1]['mean'][0], action[1]['mean'][1], action[1]['mean'][2])
        simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])    

    print "Policy Pin Score", scorer.score(simulation.cloth)
