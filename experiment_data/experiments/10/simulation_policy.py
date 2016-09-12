import numpy as np
import matplotlib.pyplot as plt
import pickle, copy, sys
import json
from os.path import dirname
sys.path.append(dirname(dirname(dirname(os.getcwd()))))
from cloth import *
from circlecloth import *
from shapecloth import *
from tensioner import *
from mouse import *
from registration import *
from scorer import *
from policy_test import *
from trpo_discrete_pin import *

"""
Presents a visualization of a learned policy.
"""

def load_policy(fname):
    """
    Return a policy from a file.
    """
    with open(fname, "rb") as f:
        try:
            return pickle.load(f)
        except EOFError:
            print 'Nothing written to file.'

def query_policy(policy, observation):
    """
    Given a policy and an observation, return an action.
    """
    observation = np.array(observation)
    return policy.get_action(observation)

def clip_action(action, low, high):
    """
    Threshold an action between two vectors, high and low.
    """
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
    policy_file = fname
    fname = "circle/policy.p"
    experiment = "circle/experiment3.json"
    simulation = load_simulation_from_config(experiment)
    policy = load_policy(fname)
    scorer = Scorer(0)
    simulation.reset()
    simulation.render = True
    pin_position, option = load_pin_from_config(experiment)
    totalpts = -scorer.score(simulation.cloth)
    print "Best possible score: ", -scorer.score(simulation.cloth)
    if mode == 'all' or mode == 'one':
        for i in range(len(simulation.trajectory)):
            simulation.update()
            simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
        print "No Pin Score", totalpts + scorer.score(simulation.cloth)

    print pin_position
    if mode == 'all' or mode == 'two':
        simulation.reset()
        tensioner = simulation.pin_position(pin_position[0], pin_position[1], option)
        for i in range(len(simulation.trajectory)):
            simulation.update()
            simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
        print "Fixed Pin Score", totalpts + scorer.score(simulation.cloth)

    if mode == 'all' or mode == 'three':
        env = PolicyGenerator("", experiment).env
        rollout(env, policy, flag=False, wait=False, render=True)
        print "Policy Pin Score", totalpts + scorer.score(simulation.cloth)
