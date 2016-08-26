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

def clip_action(action, low, high):
    action = np.array(action)
    for i in range(len(action)):
        if low[i] > action[i]:
            action[i] = low[i]
        elif action[i] > high[i]:
            action[i] = high[i]
    return action


MAPPING = {
    0 : (0,0,0),
    1 : (1,0,0),
    2 : (0,1,0),
    3 : (0,0,1),
    4 : (-1,0,0),
    5 : (0,-1,0),
    6 : (0,0,-1)
}


if __name__ == "__main__":

    if len(sys.argv) >= 2:
        mode = sys.argv[1]
    else:
        mode = 'all'
    if len(sys.argv) >= 3:
        fname = sys.argv[2]
    else:
        fname = "policy.p"
    fname = "experiment_data/" + fname

    simulation = load_simulation_from_config("config_files/experiment.json")
    policy = load_policy(fname)
    scorer = Scorer(0)
    simulation.reset()
    simulation.render = True
    simulation.trajectory = simulation.trajectory[::-1]
    pin_position, option = load_pin_from_config("config_files/experiment.json")
    print "Initial Score", scorer.score(simulation.cloth)
    print len(simulation.cloth.shapepts)
    if mode == 'all' or mode == 'one':
        total_score = 0


        for i in range(len(simulation.trajectory)):
            simulation.update()
            simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
            total_score += scorer.score(simulation.cloth)

        print "No Pin Score", scorer.score(simulation.cloth)
        print "Total Score", total_score

    if mode == 'all' or mode == 'two':
        simulation.reset()
        tensioner = simulation.pin_position(pin_position[0], pin_position[1], option)
        total_score = 0

        for i in range(len(simulation.trajectory)):
            simulation.update()
            simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
            total_score += scorer.score(simulation.cloth)

        print "Fixed Pin Score", scorer.score(simulation.cloth)
        print "Total Score", total_score

    if mode == 'all' or mode == 'three':
        simulation.reset()
        tensioner = simulation.pin_position(pin_position[0], pin_position[1], option)
        total_score = 0

        for i in range(len(simulation.trajectory)):
            simulation.update()
            action = MAPPING[query_policy(policy, [i]+list(tensioner.displacement))[0]]
            # action = clip_action(query_policy(policy, [i]+list(tensioner.displacement))[1]['mean'], [-1, -1, -1], [1, 1, 1])
            print action
            tensioner.tension(action[0], action[1], action[2])
            simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
            total_score += scorer.score(simulation.cloth)

        print "Policy Pin Score", scorer.score(simulation.cloth)
        print "Total Score", total_score
