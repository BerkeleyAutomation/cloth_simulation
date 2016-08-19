import numpy as np
import matplotlib.pyplot as plt
import pickle, copy, sys
from cloth import *
from circlecloth import *
from shapecloth import *
from tensioner import *
from mouse import *
from registration import *
from scorer import *

"""
A Simulation object that can be used to represent an ongoing experiment. It can be rendered by setting render=True on construction. See the main method for an example.
"""
class Simulation(object):

    def __init__(self, cloth, init=200, render=False):
        """
        Constructor takes in a cloth object and optionally, a nonnegative integer representing the amount of time to spend allowing
        the cloth to settle initially. Setting render=True will render the simulation. However, rendering will slow down iterations 
        by approximately 5x.
        """
        self.cloth = cloth
        self.mouse = self.cloth.mouse
        self.tensioners = self.cloth.tensioners
        self.render = render
        self.init = init
        self.bounds = cloth.bounds
        self.stored = False


    def update(self, iterations=1):
        """
        Updates the state of the cloth. Iterations signifies the amount of time to spend to allow the cloth to equilibrate.
        """
        [self.cloth.update() for _ in range(iterations)]
        if self.render:
            plt.clf()
            pts = np.array([[p.x, p.y] for p in self.cloth.normalpts])
            cpts = np.array([[p.x, p.y] for p in self.cloth.shapepts])
            if len(pts) > 0:
                plt.scatter(pts[:,0], pts[:,1], c='w')
            if len(cpts) > 0:
                plt.scatter(cpts[:,0], cpts[:,1], c='b')
            ax = plt.gca()
            plt.axis([0, 600, 0, 600])
            ax.set_axis_bgcolor('white')
            plt.pause(0.01)

    def pin_position(self, x, y):
        """
        Pins a position on the cloth.
        """
        return self.cloth.pin_position(x, y)

    def unpin_position(self, x, y):
        """
        Unpins a previously pinned position on the cloth.
        """
        self.cloth.unpin_position(x, y)

    def move_mouse(self, x, y):
        """
        Moves the mouse object.
        """
        self.mouse.move(x, y)

    def reset(self):
        """
        Resets the simulation object.
        """
        if self.render:
            plt.close()
        print "Resetting simulation."
        if not self.stored:
            self.cloth.reset()
            self.mouse = self.cloth.mouse
            self.tensioners = self.cloth.tensioners
            print "Initializing cloth"
            for i in range(self.init):
                self.cloth.update()
                if i % 10 == 0:
                    print str(i) + '/' + str(self.init)
            self.stored = copy.deepcopy(self.cloth)
            self.update(0)
        else:
            self.cloth = copy.deepcopy(self.stored)
            self.mouse = self.cloth.mouse
            self.tensioners = self.cloth.tensioners
            self.bounds = self.cloth.bounds
            self.update(0)

    def write_to_file(self, fname):
        """
        Writes a simulation object to file.
        """
        f = open(fname, "w+")
        pickle.dump(self, f)
        f.close()

    def read_from_file(fname):
        """
        Load a simuation object from file.
        """
        f = open(fname, "rb")
        try:
            return pickle.load(f)
        except EOFError:
            print 'Nothing written to file.'

    def copy(self):
        """
        Returns a deep copy of self.
        """
        return copy.deepcopy(self)


if __name__ == "__main__":
    if len(sys.argv) <= 1:
        shape_fn = lambda x, y: abs((x - 300) **2 + (y - 300) ** 2 - 150 **2) < 2000
    else:
        corners = load_robot_points()
        pts = load_robot_points("gauze_pts2.p")
        shape_fn = get_shape_fn(corners, pts, True)

    scorer = Scorer(1)

    mouse = Mouse(down=True)
    cloth = ShapeCloth(shape_fn, mouse, gravity=-2500)
    simulation = Simulation(cloth, render=True)
    simulation.reset()

    print "Initial Score", scorer.score(simulation.cloth)

    trajectory = [(np.cos(deg) * 150 + 300, np.sin(deg) * 150 + 300) for deg in [3.6 * np.pi * i / 180.0 for i in range(100)]]

    for i in range(100):
        simulation.update()
        simulation.move_mouse(trajectory[i][0], trajectory[i][1])

    print "Score", scorer.score(simulation.cloth)
    
    simulation.reset()

    simulation.pin_position(300, 300)

    for i in range(100):
        simulation.update()
        simulation.move_mouse(trajectory[i][0], trajectory[i][1])    

    print "Score", scorer.score(simulation.cloth)

