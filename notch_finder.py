import numpy as np
import matplotlib.pyplot as plt
import sys, pickle, os
from mouse import *
from shapecloth import *
from cloth import *
from circlecloth import *
from registration import *
from simulation import *
from itertools import permutations

"""
Using a heuristic, the NotchPointFinder object finds a set of segments on the 
shape function that will allow the constrained robotic arm to execute cutting
for the entire shape function. The start points of the segments are the notches.
"""

class NotchPointFinder(object):

    def __init__(self, cloth, trajectory, pin_position):
        self.cloth = cloth
        self.trajectory = trajectory
        self.isLoop = False
        self.min_pts = []
        self.max_pts = []
        self.segments = []
        self.segment_indices = []
        self.pin_position = pin_position

    def find_pts(self, armOrientation):
        """
        Returns a list of points on the shape cloth for notching.
        """
        trajectory = self.trajectory
        self.isLoop = (trajectory[0] == trajectory[-1])
        isLoop = self.isLoop

        # traverse the trajectory to find minimums and maximums

        wasMax = -1 # -1 is not seen yet, 0 is min, 1 is max
        firstpoint = []

        # in reference to an arm on the right hand side of the cloth
        if (armOrientation == "right"):
            ppt = trajectory[1] 
            pdx = trajectory[1][0] - trajectory[0][0]
            for pt in trajectory[2:]:
                dx = pt[0] - ppt[0]
                if ((pdx >= 0 and dx < 0) and (wasMax != 0)):
                    if (not firstpoint):
                        firstpoint = ppt
                    self.min_pts.append(ppt)
                    wasMax = 0
                elif (pdx <= 0 and dx > 0):
                    if (not firstpoint):
                        firstpoint = ppt
                    if (wasMax == 1 and (self.max_pts[-1][0] >= ppt[0])):
                        self.max_pts[-1] = ppt
                    elif (wasMax != 1): 
                        self.max_pts.append(ppt)
                        wasMax = 1
                ppt = pt
                pdx = dx

            # take care of end point
            if ((pdx > 0) or (pdx == 0 and wasMax == 1)):
                self.min_pts.append(ppt)
            elif ((pdx < 0) or (pdx == 0 and wasMax == 0)):
                self.max_pts.append(ppt)
            elif (pdx == 0 and wasMax == -1):
                self.max_pts.append(ppt)

            # if a loop, check for repeats at bounds of the loop
            if (isLoop):
                numMax, numMin = len(self.max_pts), len(self.min_pts)
                if (numMax < numMin):
                    if (self.min_pts[0][0] > self.min_pts[-1][0]):
                        self.min_pts.pop()
                    else:
                        self.min_pts.pop(0)
                if (numMax > numMin):
                    if (self.max_pts[0][0] < self.max_pts[-1][0]):
                        self.max_pts.pop()
                    else:
                        self.max_pts.pop(0)

        # in reference to an arm from the bottom side of the cloth
        else:
            ppt = trajectory[1] 
            pdy = trajectory[1][1] - trajectory[0][1]
            for pt in trajectory[2:]:
                dy = pt[1] - ppt[1]
                if ((pdy <= 0 and dy > 0) and (wasMax != 0)):
                    if (not firstpoint):
                        firstpoint = ppt
                    self.min_pts.append(ppt)
                    wasMax = 0
                elif (pdy >= 0 and dy < 0):
                    if (not firstpoint):
                        firstpoint = ppt
                    if (wasMax == 1 and (self.max_pts[-1][1] <= ppt[1])):
                        self.max_pts[-1] = ppt
                    elif (wasMax != 1): 
                        self.max_pts.append(ppt)
                        wasMax = 1
                ppt = pt
                pdy = dy

            # take care of end point   
            if ((pdy > 0) or (pdy == 0 and wasMax == 0)):
                self.max_pts.append(ppt)
            elif ((pdy < 0) or (pdy == 0 and wasMax == 1)):
                self.min_pts.append(ppt)
            elif (pdy == 0 and wasMax == -1):
                self.max_pts.append(ppt)

            # if a loop, check for repeats at bounds of the loop
            if (isLoop):
                numMax, numMin = len(self.max_pts), len(self.min_pts)
                if (numMax < numMin):
                    if (self.min_pts[0][1] < self.min_pts[-1][1]):
                        self.min_pts.pop()
                    else:
                        self.min_pts.pop(0)
                if (numMax > numMin):
                    if (self.max_pts[0][1] > self.max_pts[-1][1]):
                        self.max_pts.pop()
                    else:
                        self.max_pts.pop(0)        

        if (not isLoop):
            # take care of start point
            if (not firstpoint):
                if (self.max_pts):
                    self.min_pts.append(trajectory[0])
                else:
                    self.max_pts.append(trajectory[0])
            elif (firstpoint == self.max_pts[0]):
                if (armOrientation == "right"):
                    if (firstpoint[0] >= trajectory[0][0]):
                        self.max_pts[0] = trajectory[0]
                    else:
                        self.min_pts.insert(0, trajectory[0])
                else:
                    if (firstpoint[1] <= trajectory[0][1]):
                        self.max_pts[0] = trajectory[0]
                    else:
                        self.min_pts.insert(0, trajectory[0])
            else:
                if (armOrientation == "right"):
                    if (firstpoint[0] <= trajectory[0][0]):
                        self.min_pts[0] = trajectory[0]
                    else:
                        self.max_pts.insert(0, trajectory[0])
                else:
                    if (firstpoint[1] >= trajectory[0][1]):
                        self.min_pts[0] = trajectory[0]
                    else:
                        self.max_pts.insert(0, trajectory[0])

        return self.min_pts,self.max_pts

    def find_segments(self, armOrientation):
        """
        Returns a list of segments on the shape cloth for notching.
        """
        if (not self.min_pts):
            self.find_pts(armOrientation)
        
        trajectory = self.trajectory
        isLoop = self.isLoop
        length = len(trajectory)

        if (isLoop):
            i_first_min = trajectory.index(self.min_pts[0])
            i_first_max = trajectory.index(self.max_pts[0])
            if (i_first_min < i_first_max):
                # forward trajectory
                for i in range(len(self.min_pts)):
                    i_min = trajectory.index(self.min_pts[i])
                    i_max = trajectory.index(self.max_pts[i])
                    self.segments.append(trajectory[i_min:i_max+1])
                    self.segment_indices.append(range(i_min, i_max+1))
                # backward trajectory
                i_min = trajectory.index(self.min_pts[0])
                i_max = trajectory.index(self.max_pts[-1])
                self.segments.append(trajectory[:i_min+1][::-1]+trajectory[i_max:][::-1])
                self.segment_indices.append(range(0, i_min+1)[::-1]+range(i_max, length)[::-1])
                for i in range(1, len(self.min_pts)):
                    i_min = trajectory.index(self.min_pts[i])
                    i_max = trajectory.index(self.max_pts[i-1])
                    if (i_max == 0):
                        self.segments.append(trajectory[:i_min+1][::-1])
                        self.segment_indices.append(range(0, i_min+1)[::-1])
                    else:
                        self.segments.append(trajectory[i_min:i_max-1:-1])
                        self.segment_indices.append(range(i_max, i_min+1)[::-1])
            else:
                # forward trajectory
                i_min = trajectory.index(self.min_pts[-1])
                i_max = trajectory.index(self.max_pts[0])
                self.segments.append(trajectory[i_min:]+trajectory[:i_max+1])
                self.segment_indices.append(range(i_min, length) + range(0, i_max+1))
                for i in range(len(self.min_pts)-1):
                    i_min = trajectory.index(self.min_pts[i])
                    i_max = trajectory.index(self.max_pts[i+1])
                    self.segments.append(trajectory[i_min:i_max+1])
                    self.segment_indices.append(range(i_min, i_max+1))
                # backward trajectory
                for i in range(len(self.min_pts)):
                    i_min = trajectory.index(self.min_pts[i])
                    i_max = trajectory.index(self.max_pts[i])
                    if (i_max == 0):
                        self.segments.append(trajectory[:i_min+1][::-1])
                        self.segment_indices.append(range(0, i_min+1)[::-1])
                    else:
                        self.segments.append(trajectory[i_min:i_max-1:-1])
                        self.segment_indices.append(range(i_max, i_min+1)[::-1])
        else: # for non-looped trajectories
            numMin = len(self.min_pts)
            numMax = len(self.max_pts)
            if (numMin > numMax):
                for i in range(1, len(self.min_pts)):
                    i_min = trajectory.index(self.min_pts[i])
                    i_max = trajectory.index(self.max_pts[i-1])
                    self.segments.append(trajectory[i_min:i_max-1:-1])
                    self.segment_indices.append(range(i_max, i_min+1)[::-1])
                for i in range(len(self.min_pts)-1):
                    i_min = trajectory.index(self.min_pts[i])
                    i_max = trajectory.index(self.max_pts[i])
                    self.segments.append(trajectory[i_min:i_max+1])
                    self.segment_indices.append(range(i_min, i_max+1))
            elif (numMin < numMax):
                for i in range(len(self.min_pts)):
                    i_min = trajectory.index(self.min_pts[i])
                    i_max = trajectory.index(self.max_pts[i])
                    if (i_max == 0):
                        self.segments.append(trajectory[:i_min+1][::-1])
                        self.segment_indices.append(range(0, i_min+1)[::-1])
                    else:
                        self.segments.append(trajectory[i_min:i_max-1:-1])
                        self.segment_indices.append(range(i_max, i_min+1)[::-1])
                for i in range(len(self.min_pts)):
                    i_min = trajectory.index(self.min_pts[i])
                    i_max = trajectory.index(self.max_pts[i+1])
                    self.segments.append(trajectory[i_min:i_max+1])
                    self.segment_indices.append(range(i_min, i_max+1))
            else:
                for i in range(len(self.min_pts)):
                    i_min = trajectory.index(self.min_pts[i])
                    i_max = trajectory.index(self.max_pts[i])
                    if (i_min < i_max):
                        self.segments.append(trajectory[i_min:i_max+1])
                        self.segment_indices.append(range(i_min, i_max+1))
                    else:
                        if (i_max == 0):
                            self.segments.append(trajectory[:i_min+1][::-1])
                            self.segment_indices.append(range(0, i_min+1)[::-1])
                        else:
                            self.segments.append(trajectory[i_min:i_max-1:-1])
                            self.segment_indices.append(range(i_max, i_min+1)[::-1])
        return self.segments, self.segment_indices

    def find_best_trajectory(self, scorer, mode="brute"):
        """
        Returns a list of semgents in the order of the trajectory that
        corresponds to the best score.
        """
        # from simulation import *
        numSegments = len(self.segments)
        newTrajectory = []
        newIndices = []
        # brute force approach: iterate through all the permutations of segments
        if (mode == "brute"):
            permutation = list(permutations(range(0, numSegments)))
            bestScore = -10000 # arbitrary upper limit
            bestPerm = []
            simulation = Simulation(self.cloth, trajectory=None)
            for perm in permutation:
                newTrajectory = []
                newIndices = []
                for i in perm:
                    seg = self.segments[i]
                    ind = self.segment_indices[i]
                    newTrajectory = newTrajectory + seg
                    newIndices = newIndices + ind
                simulation.trajectory = newTrajectory
                simulation.reset()
                for i in range(len(simulation.trajectory)):
                    simulation.update()
                    simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
                if (scorer.score(simulation.cloth) > bestScore):
                    bestScore = scorer.score(simulation.cloth)
                    bestPerm = perm

        # length approach: order by shortest segment to longest segment
        elif (mode == "length"):
            lengths = []
            for seg in self.segments:
                lengths.append(len(seg))
            bestPerm = sorted(range(numSegments), key=lambda i: lengths[i])

        # distance approach: order by closest segment to farthest segment (average distances of all points from pin point)
        else:
            distances = []
            for seg in self.segments:
                num_pts = len(seg)
                total_sq_dist = 0
                for pt in seg:
                    dx = pt[0] - self.pin_position[0]
                    dy = pt[1] - self.pin_position[1]
                    total_sq_dist = total_sq_dist + dx**2 + dy**2
                distances.append((total_sq_dist+0.0)/num_pts)
            bestPerm = sorted(range(numSegments), key=lambda i: distances[i])
            bestPerm = bestPerm[:][::-1]

        newTrajectory = [self.segments[bestPerm[0]]]
        newIndices = [self.segment_indices[bestPerm[0]]]
        for i in bestPerm[1:]:
            seg = self.segments[i]
            ind = self.segment_indices[i]
            newTrajectory.append(seg)
            newIndices.append(ind)  
        return newTrajectory, newIndices # a list of lists


if __name__ == '__main__':

    # r is from the right, d is from the left
    armOrientation = "right"
    if len(sys.argv) > 1:
        if (sys.argv[1] == "d"):
            armOrientation = "down"
            print "Cutting from bottom to top"
        else:
            print "Cutting from right to left"

    
    mouse = Mouse(down=True, button=0)

    # Create a cloth with an interesting concave shape
    corners = [[0, 0, 0], [50, 0, 0], [0, 50, 0], [50, 50, 0]]
    points = [[7, 6, 0], [9, 5, 0], [11, 5, 0], [12, 6, 0],
              [14, 8, 0], [16, 9, 0], [18, 9, 0], [19, 8, 0],
              [20, 7, 0], [22, 5, 0], [24, 3, 0], [26, 3, 0],
              [29, 4, 0], [31, 5, 0], [34, 5, 0], [36, 7, 0],
              [38, 8, 0], [40, 11, 0], [42, 14, 0], [43, 18, 0],
              [42, 22, 0], [41, 25, 0], [40, 28, 0], [37, 31, 0],
              [34, 34, 0], [30, 36, 0], [27, 38, 0], [23, 36, 0],
              [19, 37, 0], [16, 39, 0], [12, 40, 0], [10, 36, 0],
              [8, 33, 0], [7, 31, 0], [8, 29, 0], [12, 27, 0],
              [15, 25, 0], [15, 21, 0], [12, 18, 0], [10, 14, 0],
              [7, 11, 0], [6, 9, 0], [7, 6, 0]]
    # points = list(reversed(points))
    # points = [[20, 10, 0], [40, 10, 0]]
    # points = [[40, 10, 0], [20, 10, 0]]
    # points = [[20, 10, 0], [40, 30, 0]]
    # points = [[40, 30, 0], [20, 10, 0]]
    # points = [[20, 10, 0], [25, 10, 0], [30, 15, 0], [35, 20, 0], [40, 15, 0],
    #           [42, 10, 0], [46, 10, 0]]
    # points = [[46, 10, 0], [42, 10, 0], [40, 15, 0], [35, 20, 0], [30, 15, 0],
    #           [25, 10, 0], [20, 10, 0]]
    # points = [[46, 20, 0], [42, 20, 0], [40, 15, 0], [35, 10, 0], [30, 15, 0],
    #           [25, 20, 0], [20, 20, 0]]
    # points = [[20, 40, 0], [30, 20, 0], [40, 40, 0]]
    # points = [[20, 20, 0], [30, 40, 0], [40, 20, 0]]
    # points = [[20, 20, 0], [30, 30, 0], [40, 20, 0], [30, 10, 0], [20, 20, 0]]
    # points = [[40, 20, 0], [30, 30, 0], [20, 20, 0], [30, 10, 0], [40, 20, 0]]
    # points = [[30, 30, 0], [20, 20, 0], [30, 10, 0], [40, 20, 0], [30, 30, 0]]
    # # The following two lines go together
    # corners = load_robot_points()
    # points = load_robot_points("gauze_pts2.p")
    
    shape_fn = get_shape_fn(corners, points, True)
    cloth = ShapeCloth(shape_fn, mouse)
    trajectory = get_trajectory(corners, points, True)
    
    # Find the notch points and segments to complete the trajectory
    npf = NotchPointFinder(cloth, trajectory, [600, 600])
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

    # simulate the new trajectory
    newTrajectory = []
    for seg in npf.segments:
        newTrajectory = newTrajectory + seg
    simulation = Simulation(cloth, render=True, trajectory=newTrajectory)
    scorer = Scorer(0)
    simulation.reset()
    print "Initial Score", scorer.score(simulation.cloth)
    for i in range(len(simulation.trajectory)):
        simulation.update()
        simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
    print "Score", scorer.score(simulation.cloth)
    plt.waitforbuttonpress()

    # find the best trajectory and simulate it
    newOrdering, newIndices = npf.find_best_trajectory(scorer, mode="length")
    # turn into a single list for simulation
    newTrajectory = []
    for seg in newOrdering:
        newTrajectory = newTrajectory + seg
    simulation = Simulation(cloth, render=True, trajectory=newTrajectory)
    simulation.reset()
    print "Initial Score", scorer.score(simulation.cloth)
    for i in range(len(simulation.trajectory)):
        simulation.update()
        simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
    print "Best Score", scorer.score(simulation.cloth)
    plt.waitforbuttonpress()




