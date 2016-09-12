import numpy as np
import sys, os, pickle
import IPython, ipdb
import matplotlib.pyplot as plt
from simulation import *
from tensioner import *

class AnalyticPolicy(object):

    def __init__(self, simulation, x, y):
        self.simulation = simulation
        self.tensionerx = x
        self.tensionery = y

    def rollout(self, log=False):
        self.simulation.reset()
        tensioner = self.simulation.pin_position(self.tensionerx, self.tensionery, float('inf'))
        for pos in self.simulation.trajectory:
            self.simulation.update()
            self.simulation.move_mouse(pos[0], pos[1])
            scale = 0.003
            displacement = scale * (tensioner.x - pos[0]), scale * (tensioner.y - pos[1])
            if log:
                print displacement
                print self.simulation.score
            tensioner.tension(displacement[0], displacement[1])



if __name__ == '__main__':
    if len(sys.argv) <= 1:
        shape_fn = lambda x, y: abs((x - 300) **2 + (y - 300) ** 2 - 150 **2) < 2000
    else:
        shape_fn=None

    simulation = load_simulation_from_config(shape_fn=shape_fn)
    pin_position, option = load_pin_from_config()
    print pin_position
    simulation.render = False

    ap = AnalyticPolicy(simulation, pin_position[0], pin_position[1])
    ap.rollout()
    print simulation.cloth.evaluate()

