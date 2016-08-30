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


if __name__ == '__main__':
    PLOT, NOFILTER = False, False
    args = sys.argv
    config = args[1] # must specify a config file
    if "plot" in args:
        PLOT = True
    if "nofilter" in args:
        NOFILTER = True
    config = "config_files/experiment.json"
    simulation = load_simulation_from_config(config)
    simulation.render = False
    scorer = Scorer(0)
    simulation.trajectory = simulation.trajectory[::-1]
    scores = []
    tpf = TensionPointFinder(simulation.cloth)
    pts = tpf.find_valid_pts()
    if NOFILTER:
        pts = np.ones(pts.shape)
    if PLOT:
        plt.imshow(tpf.find_valid_pts(), cmap='Greys_r')
        plt.show()
    nonzero = np.nonzero(pts)
    dx, dy = simulation.cloth.initial_params[1]
    for i in range(len(nonzero[0])):
        print i+1, "/", len(nonzero[0])
        y, x = nonzero[0][i], nonzero[1][i]
        simulation.reset()
        simulation.pin_position(dx*x+50, dy*y+50)
        for i in range(len(simulation.trajectory)):
            simulation.update()
            simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
        scores.append(scorer.score(simulation.cloth))
        print x, y, dx*x+50, dy*y+50, scores[-1]
    amax, idxmax = np.argmax(scores), np.max(scores)
    print amax, idxmax
    print 50 + nonzero[1][amax] * dx, 50 + nonzero[0][amax] * dy
    # simulation = load_simulation_from_config(config)
    # fork(simulation)
    # print 'done'
    IPython.embed()