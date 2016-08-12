import numpy as np
import matplotlib.pyplot as plt
from cloth import *
from circlecloth import *
from shapecloth import *
from tensioner import *
from mouse import *

class Simulation(object):

    def __init__(self, cloth, init=200, render=False):
        self.cloth = cloth
        self.mouse = self.cloth.mouse
        self.tensioners = self.cloth.tensioners
        print "Initializing cloth"
        for i in range(init):
            self.cloth.update()
            if i % 10 == 0:
                print str(i) + '/200'
        if render:
            plt.ion()

    def update(self, iterations=5):
        [self.cloth.update() for _ in range(iterations)]
        if render:
            plt.clf()
            pts = np.array([[p.x, p.y] for p in c.normalpts])
            cpts = np.array([[p.x, p.y] for p in c.circlepts])
            if len(pts) > 0:
                plt.scatter(pts[:,0], pts[:,1], c='w')
            if len(cpts) > 0:
                plt.scatter(cpts[:,0], cpts[:,1], c='b')
            ax = plt.gca()
            plt.axis([0, 600, 0, 600])
            ax.set_axis_bgcolor('white')
            plt.pause(0.01)

    def pin_position(self, x, y):
        return self.cloth.pin_position(x, y)

    def unpin_position(self, x, y):
        self.cloth.unpin_position(x, y)

if __name__ == "__main__":
    mouse = Mouse()
    cloth = CircleCloth(mouse)
    simulation = Simulation(cloth, render=True)

    for i in range(100):
        simulation.update()


