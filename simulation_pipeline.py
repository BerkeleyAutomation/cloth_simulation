import numpy as np
import sys, os, pickle, time
from simulation import *
from tensioner import *
from shapecloth import *
from simulation_policy import *
from registration import *
from notch_finder import *
from pattern_designer import *

"""
This file contains a script that takes user input to create a trajectory and simulates a cutting and pinning policy
Requires either a filename or (width, height, filename)
"""

if __name__ == '__main__':

    #=========================================================
    # Get points and trajectory
    #=========================================================

    if len(sys.argv) > 3:
        width = sys.argv[1]
        height = sys.argv[2]
        filename = sys.argv[3]
        pd = PatternDesigner(width, height)
    else:
        filename = sys.argv[1]
        pd = PatternDesigner()

    # Choose if you want to load this file or create a new one
    # pd.load_pts(filename)
    pd.get_pts()
    pd.save_pts(filename)

    corners = pd.corners
    points = pd.trajectory

    #=========================================================
    # Find best segment ordering and new trajectory 
    #=========================================================

    simulate = True

    mouse = Mouse(down=True, button=0)
    armOrientation = "right"
    shape_fn = get_shape_fn(corners, points, True)
    cloth = ShapeCloth(shape_fn, mouse)
    trajectory = get_trajectory(corners, points, True)

    # Find the notch points and segments to complete the trajectory
    npf = NotchPointFinder(cloth, trajectory)
    npf.find_pts(armOrientation)
    npf.find_segments(armOrientation)

    # Visualize the trajectory
    for i in range(10):
        cloth.update()
        if i % 10 == 0:
            print "Iteration", i
    fig = plt.figure()
    plt.hold(True)
    plot = fig.add_subplot(111)
    plt.clf()
    pts = np.array([[p.x, p.y] for p in cloth.normalpts])
    cpts = np.array([[p.x, p.y] for p in cloth.shapepts])
    if len(pts) > 0:
        plt.scatter(pts[:,0], pts[:,1], c='w')
    if len(cpts) > 0:
        plt.scatter(cpts[:,0], cpts[:,1], c='b')
    plt.draw()
    plt.waitforbuttonpress()

    # Visualize the mins and maxes
    minpts = np.array(npf.min_pts)
    maxpts = np.array(npf.max_pts)
    if len(minpts) > 0:
        plt.scatter(minpts[:,0], minpts[:,1], c='g', marker='s', edgecolors='none', s=80)
    if len(maxpts) > 0:
        plt.scatter(maxpts[:,0], maxpts[:,1], c='r', marker='s', edgecolors='none', s=80)
    plt.draw()
    plt.waitforbuttonpress()

    # Visualize the segments in different colors
    numSegs = len(npf.segments)
    color = iter(plt.cm.jet(np.linspace(0, 1, numSegs)))
    for i in range(numSegs):
        segpts = np.array(npf.segments[i])
        c = next(color)
        plt.scatter(segpts[:,0], segpts[:,1], c=c, marker='o', edgecolors='none', s=20)
    plt.draw()
    plt.waitforbuttonpress()

    # find the best trajectory and simulate it
    scorer = Scorer(0)
    newOrdering = npf.find_best_trajectory(scorer)
    # turn into a single list for simulation
    newTrajectory = []
    for seg in newOrdering:
        newTrajectory = newTrajectory + seg
    if (simulate):
        simulation = Simulation(cloth, render=True, trajectory=newTrajectory)
        simulation.reset()
        print "Initial Score", scorer.score(simulation.cloth)
        for i in range(len(simulation.trajectory)):
            simulation.update()
            simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
        print "Best Score", scorer.score(simulation.cloth)
        plt.waitforbuttonpress()

    trajectory = newOrdering

    #=========================================================
    # Call load_simulation_from_config in simulation.py with filename, trajectory, and boolean
    #=========================================================

    simulation = load_simulation_from_config(shape_fn=shape_fn, trajectory=trajectory, multipart=True)

    #=========================================================
    # Pinning policy
    #=========================================================


