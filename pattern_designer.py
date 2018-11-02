# import sys, pickle, os
# import numpy as np
# import matplotlib.pyplot as plt
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
from notch_finder import *

"""
An interface that allows the user to draw an arbitrary curve and returns corner points and shape points.
Takes in two arguments (width and height of the cloth grid). Default width is 50 and height is 40. 
User must click the trajectory in order, and blue dots will appear on the scatter plot. 
If you click on the same point twice, it will disappear. 
"""

class PatternDesigner(object):

    def __init__(self, width=50, height=50):
        self.width = width
        self.height = height
        self.corners = [[0, 0, 0], [width, 0, 0], [0, height, 0], [width, height, 0]]
        self.trajectory = []

    def get_pts(self):
        """
        Returns corner points and shape points
        """
        def onclick(event):
            x, y = int(event.xdata), int(event.ydata)
            if ([x, y] in self.trajectory and self.trajectory.index([x, y]) != 0):
                self.trajectory.remove([x, y])
                # kind of a hack right now, but removes the point from the scatter plot
                plt.scatter(x+0.5, y+0.5, c='w', edgecolors='w', marker='o', s=80)
                plt.draw()
            else: 
                self.trajectory.append([x, y])
            if len(np.asarray(self.trajectory)[:]) > 0:
                xs = map(lambda x : x + 0.5, np.asarray(self.trajectory)[:,0])
                ys = map(lambda y : y + 0.5, np.asarray(self.trajectory)[:,1])
                plt.scatter(xs, ys, c='b')
                fig.canvas.draw()

        def handle_close(event):
            pass
            # print('Closed Figure!')

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_xlim([0, self.width])
        ax.set_ylim([0, self.height])
        plt.xticks(np.arange(0, self.width, 1))
        plt.yticks(np.arange(0, self.height, 1))
        ax.grid(color='grey', linestyle='-')
        plt.setp(ax.get_xticklabels(), visible=False)
        plt.setp(ax.get_yticklabels(), visible=False)
        plt.scatter(24.5, 24.5, c='r')

        cid = fig.canvas.mpl_connect('button_press_event', onclick)
        fig.canvas.mpl_connect('close_event', handle_close)
        plt.show()

    def save_pts(self, filename):
        """
        Saves the corner points and shape points in an npz file in pt_files directory
        """
        for i in range(np.shape(self.trajectory)[0]):
            self.trajectory[i].append(0)
        # print self.trajectory
        np.savez('./pt_files/%s' %(filename), corners=self.corners, points=self.trajectory)


    def load_pts(self, filename):
        """
        Loads the corner points and shape points from an npz file in pt_files directory
        """
        npzfile = np.load('./pt_files/%s.npz' %(filename))
        self.corners = npzfile['corners']
        self.trajectory = npzfile['points'] 
        self.width = self.corners[1][0]
        self.height = self.corners[2][1]


if __name__ == '__main__':
    if len(sys.argv) > 3:
        width = sys.argv[1]
        height = sys.argv[2]
        filename = sys.argv[3]
        pd = PatternDesigner(width, height)
    else:
        filename = sys.argv[1]
        pd = PatternDesigner()
    # pd.load_pts(filename)
    pd.get_pts()
    pd.save_pts(filename)





