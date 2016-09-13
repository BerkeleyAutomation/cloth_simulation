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

    def rollout(self, log=False, render=False):
        self.simulation.reset()
        self.simulation.render = render
        tensioner = self.simulation.pin_position(self.tensionerx, self.tensionery, float('inf'))
        for i in range(len(self.simulation.trajectory)):
            pos = self.simulation.trajectory[i]
            if i < len(self.simulation.trajectory) - 1:
                posnext = self.simulation.trajectory[i+1]
            else:
                posnext = self.simulation.trajectory[i-1]
            self.simulation.update()
            self.simulation.move_mouse(pos[0], pos[1])
            scale = 1e-2
            displacement = np.array((tensioner.x - pos[0], tensioner.y - pos[1], 0))
            nextdisplacement = np.array((posnext[0] - pos[0], posnext[1] - pos[1], 0))
            displacement, nextdisplacement = displacement / np.linalg.norm(displacement), nextdisplacement / np.linalg.norm(nextdisplacement)
            c = np.cross(nextdisplacement, displacement)
            c = c / np.linalg.norm(c)
            dist = self.simulation.cloth.displacement_to_line(pos[0], pos[1])
            displacement = np.cross(c, nextdisplacement) * scale * dist
            if log:
                print displacement
                print self.simulation.score
            tensioner.tension(displacement[0], displacement[1])
        return self.simulation.cloth.evaluate()



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
    print ap.rollout()
    print simulation.cloth.evaluate()

