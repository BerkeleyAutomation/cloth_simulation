from tension_finder import *
from simulation import *
from shapecloth import *
from circlecloth import *
from cloth import *
from tensioner import *
from mouse import *
import numpy as np
import matplotlib.pyplot as plt
import sys, os, pickle


if __name__ == '__main__':
    PLOT = False
    args = sys.argv
    if "plot" in args:
        PLOT = True
    config = "config_files/experiment.json"
    simulation = load_simulation_from_config(config)
    simulation.render = False
    scorer = Scorer(0)
    simulation.trajectory = simulation.trajectory[::-1]
    scores = []
    tpf = TensionPointFinder(simulation.cloth)
    if PLOT:
        plt.imshow(np.flipud(tpf.find_valid_pts()), cmap='Greys_r')
        plt.show()
    nonzero = np.nonzero(tpf.find_valid_pts())
    dx, dy = simulation.cloth.initial_params[1]
    # for i in range(len(nonzero[0])):
    #     print i, "/", len(nonzero[0])
    #     x, y = nonzero[0][i], nonzero[1][i]
    #     simulation.reset()
    #     simulation.pin_position(dx*x+50, dy*y+50)
    #     for i in range(len(simulation.trajectory)):
    #         simulation.update()
    #         simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
    #     scores.append(scorer.score(simulation.cloth))
    #     print x, y, dx*x+50, dy*y+50, scores[-1]
    # print np.argmax(scores), np.max(scores)
    print nonzero[0][318]*dx + 50, nonzero[1][318] * dy + 50



