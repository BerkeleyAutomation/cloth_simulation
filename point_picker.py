from tension_finder import *
from simulation import *
from shapecloth import *
from circlecloth import *
from cloth import *
from tensioner import *
from mouse import *
import numpy as np
import matplotlib.pyplot as plt
import sys, os, pickle, copy
import IPython
from multiprocessing import *


def f(args):
    return 1
    simulation = args[0]
    y = args[1]
    x = args[2]
    simulation.render = False
    scorer = Scorer(0)
    simulation.trajectory = simulation.trajectory[::-1]
    dx, dy = simulation.cloth.initial_params[1]
    simulation.pin_position(dx*x+50, dy*y+50)
    for i in range(len(simulation.trajectory)):
        simulation.update()
        simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
    return scorer.score(simulation.cloth)


def fork(simulation, num_workers=10):

    if num_workers < 1:
        num_workers = 1
    lst = []
    tpf = TensionPointFinder(simulation.cloth)
    pts = tpf.find_valid_pts()
    nonzero = np.nonzero(pts)
    simulation.reset()
    for i in range(len(nonzero)):
        sim = copy.deepcopy(simulation)
        lst.append((sim, nonzero[0][i], nonzero[1][i]))
    p = Pool(num_workers)
    return p.map(f, tuple(lst))




class PointPicker:

    def __init__(self, config="config_files/experiment.json", NOFILTER=False):
        self.simulation = load_simulation_from_config(config)
        self.simulation.render = False
        self.scorer = Scorer(0)
        self.simulation.trajectory = self.simulation.trajectory[::-1]
        self.tpf = TensionPointFinder(self.simulation.cloth)
        self.scores = None
        self.nofilter = NOFILTER
        self.score()

    def score(self):
        if self.scores:
            return self.scores
        pts = self.tpf.find_valid_pts()
        self.scores = []
        self.nonzero = np.nonzero(pts)
        self.dx, self.dy = self.simulation.cloth.initial_params[1]
        for i in range(len(self.nonzero[0])):
            print i+1, "/", len(self.nonzero[0])
            y, x = self.nonzero[0][i], self.nonzero[1][i]
            self.simulation.reset()
            self.simulation.pin_position(self.dx*x+50, self.dy*y+50)
            for i in range(len(self.simulation.trajectory)):
                self.simulation.update()
                self.simulation.move_mouse(self.simulation.trajectory[i][0], self.simulation.trajectory[i][1])
            self.scores.append(self.scorer.score(self.simulation.cloth))
            print x, y, self.dx*x+50, self.dy*y+50, self.scores[-1]
        return self.scores

    def best_pin(self):
        amax, idxmax = np.argmax(self.scores), np.max(self.scores)
        return (50 + self.nonzero[1][amax] * self.dx, 50 + self.nonzero[0][amax] * self.dy)

    def sample_pin(self, n=10):
        norm = np.array(self.scores) - np.min(self.scores)
        norm = np.multiply(norm, norm)
        norm = np.multiply(norm, norm)
        total = np.sum(norm)
        samples = np.random.uniform(0.0, total + 1e-10, (n))
        ret = []
        for sample in samples:
            cur = 0
            for i in range(len(norm)):
                entry = norm[i]
                cur += entry
                if cur >= sample:
                    ret.append(i)
                    break
        ret_pins = []
        for elem in ret:
            ret_pins.append((50 + self.nonzero[1][elem] * self.dx, 50 + self.nonzero[0][elem] * self.dy))
            print self.scores[elem]
        return ret_pins


if __name__ == '__main__':

    p = PointPicker("experiment_data/experiments/1/experiment.json")
    print p.best_pin()

    # config = "config_files/experiment.json"
    # simulation = load_simulation_from_config(config)
    # fork(simulation)
    IPython.embed()