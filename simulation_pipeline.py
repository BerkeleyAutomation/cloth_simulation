import numpy as np
import sys, os, pickle, time
from simulation import *
from tensioner import *
from shapecloth import *
# from simulation_policy import *
from registration import *
from notch_finder import *
from tension_finder import *
from pattern_designer import *
from random import sample

"""
This file contains a script that takes user input to create a trajectory and simulates a cutting and pinning policy
Requires either a filename or (width, height, filename)
"""

if __name__ == '__main__':

    #=========================================================
    # Setup
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
    pd.load_pts(filename)
    # pd.get_pts()
    # pd.save_pts(filename)
   
    corners = pd.corners
    points = pd.trajectory
 
    # corners = load_robot_points(fname="config_files/gauze_pts.p")
    # points = load_robot_points(fname="config_files/gauze_pts2.p")

    mouse = Mouse(down=True, button=0)
    armOrientation = "right"
    shape_fn = get_shape_fn(corners, points, True)
    cloth = ShapeCloth(shape_fn, mouse, width=25, height=25, dx=20, dy=20)
    trajectory = get_trajectory(corners, points, True)

    #=========================================================
    # Find the best segment trajectory to pass to pinning
    #=========================================================

    simulate = False

    # Find the notch points and segments to complete the trajectory
    npf = NotchPointFinder(cloth, trajectory)
    npf.find_pts(armOrientation)
    npf.find_segments(armOrientation)

    # # Visualize the trajectory
    # for i in range(10):
    #     cloth.update()
    #     if i % 10 == 0:
    #         print "Iteration", i
    # fig = plt.figure()
    # plt.hold(True)
    # plot = fig.add_subplot(111)
    # plt.clf()
    # pts = np.array([[p.x, p.y] for p in cloth.normalpts])
    # cpts = np.array([[p.x, p.y] for p in cloth.shapepts])
    # if len(pts) > 0:
    #     plt.scatter(pts[:,0], pts[:,1], c='w')
    # if len(cpts) > 0:
    #     plt.scatter(cpts[:,0], cpts[:,1], c='b')
    # plt.draw()

    # # Visualize the mins and maxes
    # minpts = np.array(npf.min_pts)
    # maxpts = np.array(npf.max_pts)
    # if len(minpts) > 0:
    #     plt.scatter(minpts[:,0], minpts[:,1], c='g', marker='s', edgecolors='none', s=80)
    # if len(maxpts) > 0:
    #     plt.scatter(maxpts[:,0], maxpts[:,1], c='r', marker='s', edgecolors='none', s=80)
    # plt.draw()
    # plt.waitforbuttonpress()

    # # Visualize the segments in different colors
    # numSegs = len(npf.segments)
    # color = iter(plt.cm.jet(np.linspace(0, 1, numSegs)))
    # for i in range(numSegs):
    #     segpts = np.array(npf.segments[i])
    #     c = next(color)
    #     plt.scatter(segpts[:,0], segpts[:,1], c=c, marker='o', edgecolors='none', s=20)
    # plt.draw()
    # plt.waitforbuttonpress()

    # find the best trajectory and simulate it
    scorer = Scorer(0)
    newOrdering, newIndices, best_score, worst_score = npf.find_best_trajectory(scorer)

    old_trajectory = []
    for seg in npf.segments:
        old_trajectory = old_trajectory + seg

    simulation = Simulation(cloth, render=simulate, trajectory=newOrdering, multi_part=True)
    simulation.reset()
    totalpts = len(simulation.cloth.shapepts)
    init_score = scorer.score(simulation.cloth) + totalpts

    print "Initial Score", init_score
    print "Best Score", best_score + totalpts
    print "Worst Score", worst_score + totalpts
    
    # save results
    f = open("sim_files/%s/nohold" %(filename), "w+")
    data = {'totalpts': totalpts, 'init_score': init_score, 'best_score': best_score+totalpts, 'worst_score': worst_score+totalpts,
            'old_trajectory': old_trajectory, 'trajectory': newOrdering, 'indices_of_pts': newIndices}
    pickle.dump(data, f)
    f.close()

    # plt.waitforbuttonpress()

    #########################################

    # 10 times to get std error
    with open("sim_files/%s/nohold" %(filename), "rb") as f:
        try:
            data = pickle.load(f)
        except EOFError:
            print 'Nothing written to file.'
    totalpts = data['totalpts']
    init_score = data['init_score']
    old_trajectory = data['old_trajectory']
    new_trajectory = data['trajectory']
    score = []

    # simulation = Simulation(cloth, render=True, trajectory=new_trajectory, multi_part=True)
    # simulation.reset()
    # totalpts = len(simulation.cloth.shapepts)
    # init_score = scorer.score(simulation.cloth) + totalpts
    # print "Initial Score", init_score
    # for i in range(len(simulation.trajectory)):
    #     simulation.update()
    #     simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
    # print scorer.score(simulation.cloth) + totalpts

    for t in range(1,11):
        scorer = Scorer(0)

        simulation = Simulation(cloth, render=simulate, trajectory=new_trajectory, multi_part=True)
        simulation.reset()

        for i in range(len(simulation.trajectory)):
            simulation.update()
            simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
        score.append(scorer.score(simulation.cloth) + totalpts) 
        print "Score %d" %(t), scorer.score(simulation.cloth) + totalpts

    # save results
    f = open("sim_files/%s/noholds" %(filename), "w+")
    data = {'totalpts': totalpts, 'init_score': init_score, 'scores': score}
    pickle.dump(data, f)
    f.close()

    #===============================================================
    # Find best segment ordering now with pinning and new trajectory 
    #===============================================================

    # find valid points and sample these
    tpf = TensionPointFinder(cloth)
    pin_pts = tpf.find_valid_pts(allpts=True).nonzero()  
    lst = []
    for i in range(len(pin_pts[0])):
        lst.append([pin_pts[0][i]*10+50, pin_pts[1][i]*10+50])
    pin_pts = lst
    # sample 30 of these points uniformly
    indices = sample(xrange(len(pin_pts)), 30) 
    pts_to_test = [pin_pts[i][:] for i in indices]

    # find the best pinning position (and the worst)
    npf = NotchPointFinder(cloth, trajectory)
    npf.find_pts(armOrientation)
    npf.find_segments(armOrientation)
    bestScore = -10000 # arbitrary upper limit
    worstScore = 10000
    best_pin_pt = []
    worst_pin_pt = []
    bestOrdering = []
    bestIndices = []
    scores = []
    print len(pts_to_test)
    for pin_pt in pts_to_test:
        npf.pin_position = pin_pt
        scorer = Scorer(0)
        newOrdering, newIndices, best_score, worst_score = npf.find_best_trajectory(scorer)
        scores.append(best_score + totalpts)
        if (best_score > bestScore):
            bestScore = best_score
            best_pin_pt = pin_pt
            bestOrdering = newOrdering
            bestIndices = newIndices
        if (worst_score < worstScore):
            worstScore = worst_score
            worst_pin_pt = pin_pt

    # get some variables to save
    scorer = Scorer(0)
    simulation = Simulation(cloth, render=simulate, trajectory=bestOrdering, multi_part=True)
    simulation.reset()
    totalpts = len(simulation.cloth.shapepts)
    init_score = scorer.score(simulation.cloth) + totalpts
    old_trajectory = []
    for seg in npf.segments:
        old_trajectory = old_trajectory + seg

    # save results
    f = open("sim_files/%s/hold" %(filename), "w+")
    data = {'totalpts': totalpts, 'init_score': init_score, 'best_score': bestScore+totalpts, 'worst_score': worstScore+totalpts,
            'old_trajectory': old_trajectory, 'trajectory': bestOrdering, 'indices_of_pts': bestIndices,
            'best_pin_pt': best_pin_pt, 'worst_pin_pt': worst_pin_pt, 'pin_pts': pts_to_test, 'pin_scores': scores}
    pickle.dump(data, f)
    f.close()

    # 10 times to get std error
    with open("sim_files/%s/hold" %(filename), "rb") as f:
        try:
            data = pickle.load(f)
        except EOFError:
            print 'Nothing written to file.'
    totalpts = data['totalpts']
    init_score = data['init_score']
    old_trajectory = data['old_trajectory']
    new_trajectory = data['trajectory']
    best_pin_pt = data['best_pin_pt']
    score = []

    # simulation = Simulation(cloth, render=True, trajectory=new_trajectory, multi_part=True)
    # simulation.reset()
    # simulation.pin_position(best_pin_pt[0], best_pin_pt[1])
    # totalpts = len(simulation.cloth.shapepts)
    # init_score = scorer.score(simulation.cloth) + totalpts
    # print "Initial Score", init_score
    # for i in range(len(simulation.trajectory)):
    #     simulation.update()
    #     simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
    # print scorer.score(simulation.cloth) + totalpts

    for t in range(1,11):
        scorer = Scorer(0)

        simulation = Simulation(cloth, render=simulate, trajectory=new_trajectory, multi_part=True)
        simulation.reset()
        simulation.pin_position(best_pin_pt[0], best_pin_pt[1])

        for i in range(len(simulation.trajectory)):
            simulation.update()
            simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
        score.append(scorer.score(simulation.cloth) + totalpts) 
        print "Score %d" %(t), scorer.score(simulation.cloth) + totalpts

    # save results
    f = open("sim_files/%s/holds" %(filename), "w+")
    data = {'totalpts': totalpts, 'init_score': init_score, 'scores': score}
    pickle.dump(data, f)
    f.close()





