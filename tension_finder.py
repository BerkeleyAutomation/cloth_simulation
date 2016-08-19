import numpy as np
import matplotlib.pyplot as plt
import sys, pickle, os
from mouse import *
from shapecloth import *
from cloth import *
from registration import *
from scipy import signal
from scipy import stats

"""
Using a heuristic, the TensionPointFinder object finds a set of valid points on the cloth that could be grabbed by a tensioner and stilla avoid collisions.
"""

class TensionPointFinder(object):

    def __init__(self, cloth):
        self.cloth = cloth

    def find_valid_pts(self):
        """
        Returns a map of valid points on the cloth object that could be used to tensioned without collisions with the second arm.
        """
        width, height = self.cloth.initial_params[0]
        dx, dy = self.cloth.initial_params[1]
        shape_fn = self.cloth.initial_params[2]
        grid = np.zeros((height, width))
        for i in range(height):
            for j in range(width):
                if shape_fn(j * dy + 50, i * dx + 50):
                    grid[i, j] = 1
        plt.imshow(-np.flipud(grid) + 1, cmap='Greys_r')
        plt.show()
        lock = False
        continued = False
        for i in range(height):
            lock = False
            for j in range(width):
                if lock:
                    grid[i, j] = 1
                elif grid[i, j]:
                    lock = True
        lock = False
        centery = int(self.cloth.initial_params[0][1] / 2)
        for i in reversed(range(height)):
            for j in reversed(range(width)):
                if not grid[i, j]:
                    lock = False
                    s = self.slope(j, i, -50)
                    for x in range(width):
                        y = min(max(int(centery + s * x), 0), height - 1)
                        if lock:
                            grid[y, x] = 1
                        elif grid[y, x]:
                            lock = True
        grid = signal.convolve2d(grid, np.ones((5, 5)))
        grid = stats.threshold(grid, threshmax=1e-10, newval=1)
        return -grid + 1

    def slope(self, x, y, centerx=None, centery=None):
        """
        Computes slope from (centerx, centery) to (x, y).
        """
        if not centerx:
            centerx = 0
        if not centery:
                centery = int(self.cloth.initial_params[0][1] / 2)
        return (y - centery) / (x - centerx + 1e-10)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        shape_fn = lambda x, y: abs((x - 300) **2 + (y - 300) ** 2 - 150 **2) < 2000
    else:
        corners = load_robot_points()
        pts = load_robot_points("gauze_pts2.p")
        shape_fn = get_shape_fn(corners, pts, True)


    mouse = Mouse(down=True)
    cloth = ShapeCloth(shape_fn, mouse)
    tpf = TensionPointFinder(cloth)
    plt.imshow(np.flipud(tpf.find_valid_pts()), cmap='Greys_r')
    plt.show()
